function u = yig_controller(f_target, y, P)
%YIG_CONTROLLER  YOUR CONTROL LAW GOES HERE.
%
%   This function is what the "YIG Controller" MATLAB Function block in
%   yig_tuning_model.slx calls every controller step (P.sim.Tc).  Edit this
%   file (or point the block at a different file, see README) and just
%   re-run the simulation -- no need to touch the Simulink diagram.
%
%   Rules for anything you write in here (MATLAB Function block = codegen):
%     - keep state in PERSISTENT variables, initialise them inside
%       "if isempty(...)"
%     - fixed-size arrays only; no strings, cells, plotting, or disp in
%       the hot path
%     - everything you need from the outside world comes in through y and P
%
%   INPUTS
%     f_target  desired filter centre frequency [Hz]
%     y (8x1)   measurements from the plant (one plant-sample old)
%        y(1) I_meas     main-coil current [A]
%        y(2) Vc_meas    storage-cap voltage [V]
%        y(3) B_hall     Hall-sensor reading [T]  -> hall_to_f() gives frequency
%        y(4) P_det      detector power at the probe frequency [dBm]
%        y(5) T_meas     magnet temperature [degC]
%        y(6) busy       1 while a write pulse is in progress
%        y(7) ready      1 when idle and cap is charged to V0_set
%        y(8) Itrim_meas trim-coil current [A]
%     P         the parameter struct from yig_params.m
%
%   OUTPUT  u (8x1)
%        u(1) V0_set     charger target voltage [V]
%        u(2) fire       0 -> 1 edge fires one pulse
%        u(3) polarity   +1 (field up, "write") / -1 (field down, "reset")
%        u(4) I_lim      peak-current cutoff for the pulse [A] (0 = none)
%        u(5) I_trim     trim-coil current command [A]
%        u(6) f_probe    probe-tone frequency for the detector [Hz]
%        u(7) hall_on    1 while the Hall sensor is powered (power accounting)
%        u(8) mode       controller state id (logging only)
%
%   ------------------------------------------------------------------
%   BASELINE STRATEGY (the vendor procedure, closed around the Hall sensor).
%   P.ctrl.strategy picks how the magnet is fine-tuned:
%     1 = APPROACH FROM BELOW: aim writes low, refine with small + pulses;
%         a magnet that ends up too high needs a reset + rewrite.
%     2 = OVERSHOOT, CORRECT DOWN: aim writes high, refine with small
%         REVERSE (-) pulses, which move the field 4-6x more gently than
%         + writes; a magnet that ends up too low is re-written at the same
%         voltage (restoring the overshoot) and corrected down again.
%
%     1. RESET  : natural pulse at -V_reset -> magnet at its lowest field.
%     2. WRITE  : natural pulse at +V_k, V_k = LUT(f_target -/+ write_bias).
%     3. MEASURE: after the eddy currents settle, estimate the magnet-only
%                 frequency. Within the accept window -> feed the remaining
%                 error straight into the trim coil (feedforward) and TRACK.
%                 Otherwise: too low -> bigger write (no reset needed,
%                 Preisach wipe-out); too high -> reset + smaller write.
%     4. TRACK  : PI on the trim coil holds f_target.
%     5. STANDBY: after t_standby, refine pulses (see strategy) move the
%                 magnet until the trim supplies < standby_enter_tol, then
%                 trim and Hall turn off (zero-power hold). Every t_wake the
%                 Hall is powered briefly; if the error exceeds
%                 standby_wake_tol, go back to TRACK (which refines again).
%   The voltage LUT is refined online after every write.
%   ------------------------------------------------------------------

% mode ids
RESET_CHARGE = 1;  PULSE_WAIT = 2;  WRITE_CHARGE = 3;  MEASURE = 4;
TRACK = 5;  STANDBY = 6;  WAKE = 7;  REV_CHARGE = 8;
% what the last pulse was (decides how MEASURE uses the result)
K_WRITE = 1;  K_REV = 2;  K_RESTORE = 3;

persistent mode next_mode timer f_act V_k V_lo V_hi has_lo has_hi ...
           V_prev f_prev has_prev iter V_max_since_reset f_acc n_acc ...
           integ sat_timer cal_V dVdf0 dVdf fire_out V0 pol ...
           trim_on hall_on hall_timer hold_timer wake_timer n_nudge aim_bias ...
           kind V_w f_w V_rev V_nl f_nl V_a d_a V_a2 d_a2 V_b d_b dz_step
if isempty(mode)
    kind = 1;  V_w = 0;  f_w = 0;  V_rev = 0;  V_nl = 0;  f_nl = 0;
    V_a = 0;  d_a = 0;  V_a2 = 0;  d_a2 = 0;  V_b = 0;  d_b = 0;  dz_step = 0;
    cal_V = P.ctrl.cal_V(:);
    dVdf0 = (cal_V(end) - cal_V(1)) / (P.ctrl.cal_f(end) - P.ctrl.cal_f(1));
    f_act = f_target;
    V_lo = 0;  V_hi = 0;  V_prev = 0;  f_prev = 0;  has_prev = 0;
    V_max_since_reset = 0;  f_acc = 0;  n_acc = 0;  sat_timer = 0;
    fire_out = 0;  V0 = 0;  pol = 1;  hall_timer = 0;
    hold_timer = 0;  wake_timer = 0;  next_mode = WRITE_CHARGE;
    mode = 0;  timer = 0;  iter = 0;  has_lo = 0;  has_hi = 0;
    integ = 0;  V_k = 0;  dVdf = dVdf0;  trim_on = 0;  hall_on = 1;
    n_nudge = 0;  aim_bias = 0;
    [mode, timer, iter, has_lo, has_hi, has_prev, integ, V_k, dVdf, trim_on, aim_bias] = ...
        start_write(f_act, P, cal_V, dVdf0, RESET_CHARGE);
end

Tc = P.sim.Tc;
timer = timer + Tc;
fire_out = 0;
trim_gain  = P.trim.kB*P.yig.gamma;          % [Hz/A]
trim_range = P.trim.Imax*trim_gain;          % [Hz]

busy = y(6);  ready = y(7);
if hall_on > 0.5, hall_timer = hall_timer + Tc; else, hall_timer = 0; end
hall_ok = hall_on > 0.5 && hall_timer > P.ctrl.t_hall_on;
f_meas = hall_to_f(y(3), y(5), P);
f_mag  = f_meas - y(8)*trim_gain;            % magnet-only frequency

% ---------- new target? ----------------------------------------------
if abs(f_target - f_act) > P.ctrl.retarget_tol
    trim_needed = trim_on*integ*trim_gain + (f_target - f_act);
    holding = (mode == TRACK) || (mode == STANDBY) || (mode == WAKE);
    small_move = holding && (abs(trim_needed) < 0.5*trim_range);
    f_act = f_target;
    if small_move
        if trim_on < 0.5, integ = 0; end
        mode = TRACK;  trim_on = 1;  hall_on = 1;
        hold_timer = 0;  n_nudge = 0;
    else
        [mode, timer, iter, has_lo, has_hi, has_prev, integ, V_k, dVdf, trim_on, aim_bias] = ...
            start_write(f_act, P, cal_V, dVdf0, RESET_CHARGE);
        hall_on = 1;  n_nudge = 0;  kind = K_WRITE;
    end
end

switch mode
    case RESET_CHARGE  % ---- charge to V_reset, fire a - pulse ---------
        V0 = P.ctrl.V_reset;  pol = -1;
        if timer > 2*Tc && ready > 0.5 && busy < 0.5
            fire_out = 1;  mode = PULSE_WAIT;  next_mode = WRITE_CHARGE;
            timer = 0;  V_max_since_reset = 0;
        end

    case PULSE_WAIT    % ---- wait for the pulse to finish ---------------
        if timer > 2*Tc && busy < 0.5
            mode = next_mode;  timer = 0;  f_acc = 0;  n_acc = 0;
        end

    case WRITE_CHARGE  % ---- charge to V_k, fire a + pulse -------------
        V0 = V_k;  pol = 1;
        if timer > 2*Tc && ready > 0.5 && busy < 0.5
            fire_out = 1;  mode = PULSE_WAIT;  next_mode = MEASURE;
            timer = 0;  V_max_since_reset = max(V_max_since_reset, V_k);
        end

    case REV_CHARGE    % ---- charge to V_rev, fire a small - pulse -----
        V0 = V_rev;  pol = -1;
        if timer > 2*Tc && ready > 0.5 && busy < 0.5
            fire_out = 1;  mode = PULSE_WAIT;  next_mode = MEASURE;  timer = 0;
        end

    case MEASURE       % ---- let eddy currents settle, average f -------
        V0 = P.ctrl.V_idle;
        if timer > P.ctrl.t_settle && hall_ok
            f_acc = f_acc + f_mag;  n_acc = n_acc + 1;
            if n_acc >= P.ctrl.n_avg
                f_k = f_acc / n_acc;
                f_aim = f_act - aim_bias;
                iter = iter + 1;
                % ---- local slope dV/df from the last two writes ----
                % (only from steps well above the charger noise; a secant
                %  over a few mV is mostly noise)
                if has_prev > 0.5 && abs(V_k - V_prev) > 0.2 && abs(f_k - f_prev) > 1e6
                    s = (V_k - V_prev)/(f_k - f_prev);
                    if s > 0.2*dVdf0 && s < 5*dVdf0
                        dVdf = s;
                    end
                end
                V_prev = V_k;  f_prev = f_k;  has_prev = 1;
                if P.ctrl.strategy == 2
                    % accept from ABOVE (or barely below): reverse pulses lower it later
                    in_window = (f_k - f_act) < P.ctrl.write_tol && ...
                                (f_act - f_k) <= P.ctrl.standby_enter_tol;
                else
                    % accept from BELOW (or barely above): raising later needs no reset
                    in_window = (f_act - f_k) < P.ctrl.write_tol && ...
                                (f_k - f_act) <= P.ctrl.standby_enter_tol;
                end
                if kind ~= K_WRITE
                    in_window = abs(f_k - f_act) < P.ctrl.write_tol;   % refine pulses
                end
                if in_window || iter >= P.ctrl.max_iter
                    % ---- done: learn, feed the rest into the trim ------
                    if kind == K_REV
                        V_nl = V_rev;  f_nl = f_k;
                        if f_k > f_act                 % not far enough: new lower bound
                            V_a2 = V_a;  d_a2 = d_a;  V_a = V_rev;  d_a = f_w - f_k;
                        end
                    else                               % + write or restore
                        if kind == K_WRITE
                            V_at_target = V_k + (f_act - f_k)*dVdf;
                            cal_V = cal_update(P.ctrl.cal_f(:), cal_V, f_act, V_at_target);
                            V_a = 0;  d_a = 0;  V_a2 = 0;  d_a2 = 0;  V_b = 0;  d_b = 0;  dz_step = 0;   % new reverse search
                        end
                        V_w = V_k;  f_w = f_k;  V_nl = 0;  f_nl = 0;
                    end
                    integ = min(max((f_act - f_k)/trim_gain, -P.trim.Imax), P.trim.Imax);
                    trim_on = 1;  mode = TRACK;  timer = 0;  sat_timer = 0;
                    if n_nudge > 0
                        hold_timer = P.ctrl.t_standby - P.ctrl.t_refine;   % refine again sooner
                    else
                        hold_timer = 0;
                    end
                elseif kind == K_REV && f_k < f_act
                    % reverse pulse went too far: new upper bound; restore the
                    % overshoot write, then come down again in between
                    V_b = V_rev;  d_b = f_w - f_k;
                    V_k = V_w;  kind = K_RESTORE;  iter = 0;  timer = 0;  mode = WRITE_CHARGE;
                elseif kind ~= K_WRITE
                    [mode, timer, iter, has_lo, has_hi, has_prev, integ, V_k, dVdf, trim_on, aim_bias] = ...
                        start_write(f_act, P, cal_V, dVdf0, RESET_CHARGE);
                    kind = K_WRITE;
                else
                    % ---- bracket (f increases with V) and next guess ----
                    if f_k > f_aim
                        V_hi = V_k;  has_hi = 1;
                    else
                        V_lo = V_k;  has_lo = 1;
                    end
                    V_new = V_k + (f_aim - f_k)*dVdf;          % secant step
                    if has_lo > 0.5 && has_hi > 0.5
                        w = V_hi - V_lo;
                        if V_new <= V_lo + 0.02*w || V_new >= V_hi - 0.02*w
                            V_new = 0.5*(V_lo + V_hi);         % bisection fallback
                        end
                    end
                    V_k = min(max(V_new, 0), P.ctrl.V_write_max);
                    timer = 0;
                    if V_k > V_max_since_reset
                        mode = WRITE_CHARGE;   % bigger write: no reset needed
                    else
                        mode = RESET_CHARGE;   % smaller write: reset first
                        trim_on = 0;  integ = 0;
                    end
                end
            end
        end

    case TRACK         % ---- trim PI holds; head for standby ----------
        V0 = P.ctrl.V_idle;  pol = 1;
        hold_timer = hold_timer + Tc;
        if abs(integ) > P.ctrl.trim_frac*P.trim.Imax
            sat_timer = sat_timer + Tc;
        else
            sat_timer = 0;
        end
        if sat_timer > P.ctrl.t_trim_sat           % out of trim range -> rewrite
            [mode, timer, iter, has_lo, has_hi, has_prev, integ, V_k, dVdf, trim_on, aim_bias] = ...
                start_write(f_act, P, cal_V, dVdf0, RESET_CHARGE);
            sat_timer = 0;  kind = K_WRITE;
        elseif hold_timer > P.ctrl.t_standby && hall_ok
            trim_Hz = integ*trim_gain;             % what the trim supplies now
            hold_timer = 0;
            stuck = (n_nudge >= P.ctrl.max_nudge + (P.ctrl.strategy == 2)*P.ctrl.max_rev) || ...
                    (P.ctrl.strategy == 2 && V_b > 0 && (V_b - V_a) < 0.1);   % bracket too narrow
            if abs(trim_Hz) < P.ctrl.standby_enter_tol || (stuck && abs(trim_Hz) <= P.ctrl.spec_tol)
                mode = STANDBY;  trim_on = 0;  integ = 0;  hall_on = 0;  wake_timer = 0;
            elseif stuck && P.ctrl.strategy == 2   % refinement not converging: start over
                [mode, timer, iter, has_lo, has_hi, has_prev, integ, V_k, dVdf, trim_on, aim_bias] = ...
                    start_write(f_act, P, cal_V, dVdf0, RESET_CHARGE);
                kind = K_WRITE;  n_nudge = 0;
            elseif n_nudge < P.ctrl.max_nudge + (P.ctrl.strategy == 2)*P.ctrl.max_rev
                n_nudge = n_nudge + 1;
                iter = 0;  timer = 0;
                if trim_Hz > 0 && P.ctrl.strategy == 2 && V_nl > 0 && f_w > f_act + P.ctrl.standby_enter_tol
                    % too low after a reverse pulse: upper bound; restore the overshoot
                    V_b = V_nl;  d_b = f_w - f_nl;
                    V_k = V_w;  kind = K_RESTORE;  mode = WRITE_CHARGE;
                elseif trim_Hz > 0                 % magnet too low: small + nudge
                    % half step: undershooting only costs another nudge
                    dV = P.ctrl.nudge_gain*trim_Hz*local_slope(P.ctrl.cal_f(:), cal_V, f_act, dVdf0);
                    V_k = min(V_k + dV, P.ctrl.V_write_max);
                    aim_bias = 0;  kind = K_WRITE;
                    mode = WRITE_CHARGE;           % trim keeps holding meanwhile
                elseif P.ctrl.strategy == 2        % magnet too high: small reverse pulse
                    D = f_w - f_act;               % drop needed from the overshoot write
                    if V_b > 0                     % bracketed: secant, kept inside (V_a, V_b)
                        w = V_b - V_a;
                        V_rev = V_a + (D - d_a)*w/max(d_b - d_a, 1);
                        V_rev = min(max(V_rev, V_a + 0.1*w), V_b - 0.1*w);
                    elseif V_a > 0 && d_a > 2e6    % on the curve: local slope, half step, <= 1 V
                        sl = 0;
                        if V_a2 > 0 && V_a > V_a2, sl = (d_a - d_a2)/(V_a - V_a2); end
                        if sl > 1e6
                            dV = P.ctrl.nudge_gain*(D - d_a)/sl;
                        else
                            dV = 0.25;
                        end
                        V_rev = V_a + min(max(dV, 0.05), 1.0);
                    elseif V_a > 0                 % still in the dead zone: growing steps
                        dz_step = max(2*dz_step, P.ctrl.rev_step0);
                        V_rev = V_a + min(dz_step, P.ctrl.rev_dV_max);
                    else                           % first try: reverse calibration table
                        V_rev = rev_lookup(P, V_w, D);
                    end
                    V_rev = min(V_rev, P.ctrl.V_reset);
                    kind = K_REV;  mode = REV_CHARGE;
                else                               % strategy 1, too high: reset + rewrite
                    [mode, timer, iter, has_lo, has_hi, has_prev, integ, V_k, dVdf, trim_on, aim_bias] = ...
                        start_write(f_act, P, cal_V, dVdf0, RESET_CHARGE);
                    kind = K_WRITE;
                end
            end
        end

    case STANDBY       % ---- zero-power hold ---------------------------
        V0 = 0;  trim_on = 0;  hall_on = 0;
        wake_timer = wake_timer + Tc;
        if wake_timer > P.ctrl.t_wake
            mode = WAKE;  hall_on = 1;  timer = 0;  f_acc = 0;  n_acc = 0;
        end

    otherwise          % ---- WAKE: brief Hall check ---------------------
        V0 = 0;  trim_on = 0;  hall_on = 1;
        if hall_ok
            f_acc = f_acc + f_meas;  n_acc = n_acc + 1;
            if n_acc >= P.ctrl.n_avg
                if abs(f_acc/n_acc - f_act) > P.ctrl.standby_wake_tol
                    mode = TRACK;  trim_on = 1;  integ = 0;  hold_timer = 0;  n_nudge = 0;
                else
                    mode = STANDBY;  hall_on = 0;  wake_timer = 0;
                end
            end
        end
end

% ---------- trim PI (frozen during pulses and while the Hall warms up) ---
if trim_on > 0.5 && busy < 0.5 && hall_ok && mode == TRACK
    integ = integ + P.ctrl.Ki_trim*(f_act - f_meas)*Tc;
    integ = min(max(integ, -P.trim.Imax), P.trim.Imax);
end
if trim_on > 0.5
    pterm = 0;
    if mode == TRACK && hall_ok, pterm = P.ctrl.Kp_trim*(f_act - f_meas); end
    I_trim = min(max(integ + pterm, -P.trim.Imax), P.trim.Imax);
else
    I_trim = 0;
end

u = [V0; fire_out; pol; 0; I_trim; f_act; hall_on; mode];   % I_lim = 0: natural pulses only
end

%% ======================================================================
function f = hall_to_f(B_hall, T, P)
% firmware conversion: temperature-compensate the gain, apply the bench cal
Bc = B_hall / (1 + P.ctrl.hall_tc_comp*(T - P.ctrl.T_cal));
f  = P.ctrl.hall_a*Bc + P.ctrl.hall_b;
end

function [mode, timer, iter, has_lo, has_hi, has_prev, integ, V_k, dVdf, trim_on, aim_bias] = ...
         start_write(f_t, P, cal_V, dVdf0, RESET_CHARGE)
mode = RESET_CHARGE;  timer = 0;  iter = 0;
has_lo = 0;  has_hi = 0;  has_prev = 0;  integ = 0;  trim_on = 0;
aim_bias = P.ctrl.write_bias;                 % strategy 1: aim low
if P.ctrl.strategy == 2, aim_bias = -P.ctrl.write_bias; end   % strategy 2: aim high
V_k  = min(max(cal_lookup(P.ctrl.cal_f(:), cal_V, f_t - aim_bias), 0), P.ctrl.V_write_max);
dVdf = local_slope(P.ctrl.cal_f(:), cal_V, f_t, dVdf0);
end

function V = rev_lookup(P, V_w, D)
% reverse voltage that lowers f by D after a write at V_w (bilinear in the
% calibration table, clamped to its range)
Vg = P.ctrl.rev_Vw(:);  Dg = P.ctrl.rev_D(:);  T = P.ctrl.rev_V;
i = seg(Vg, V_w);  a = min(max((V_w - Vg(i))/(Vg(i+1) - Vg(i)), 0), 1);
j = seg(Dg, D);    b = min(max((D - Dg(j))/(Dg(j+1) - Dg(j)), 0), 1);
V = (1-a)*(1-b)*T(i,j) + a*(1-b)*T(i+1,j) + (1-a)*b*T(i,j+1) + a*b*T(i+1,j+1);
end

function I = cal_lookup(cf, cI, f)
% linear interpolation with linear extrapolation at the ends
j = seg(cf, f);
t = (f - cf(j)) / (cf(j+1) - cf(j));
I = (1 - t)*cI(j) + t*cI(j+1);
end

function s = local_slope(cf, cV, f, s0)
% dV/df of the table segment that contains f (fallback s0)
j = seg(cf, f);
s = (cV(j+1) - cV(j)) / (cf(j+1) - cf(j));
if ~(s > 0.2*s0 && s < 5*s0), s = s0; end
end

function j = seg(cf, f)
n = numel(cf);
if f <= cf(1)
    j = 1;
elseif f >= cf(n)
    j = n - 1;
else
    j = 1;
    while j < n - 1 && f > cf(j+1)
        j = j + 1;
    end
end
end

function cI = cal_update(cf, cI, f, I_true)
% nudge the two neighbouring nodes so the table hits (f, I_true) exactly
n = numel(cf);
fc = min(max(f, cf(1)), cf(n));
j = seg(cf, fc);
t = (fc - cf(j)) / (cf(j+1) - cf(j));
w1 = 1 - t;  w2 = t;
delta = I_true - cal_lookup(cf, cI, fc);
s = w1^2 + w2^2;
cI(j)   = cI(j)   + delta*w1/s;
cI(j+1) = cI(j+1) + delta*w2/s;
end
