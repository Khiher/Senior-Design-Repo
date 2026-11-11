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
%        y(3) f_meas     filter centre-frequency estimate [Hz]
%        y(4) P_det      detector power at the probe frequency [dBm]
%        y(5) T_meas     magnet temperature [degC]
%        y(6) busy       1 while a write pulse is in progress
%        y(7) ready      1 when idle and cap is charged to V0_set
%        y(8) Itrim_meas trim-coil current [A]
%     P         the parameter struct from yig_params.m
%
%   OUTPUT  u (6x1)
%        u(1) V0_set     charger target voltage [V]
%        u(2) fire       0 -> 1 edge fires one pulse
%        u(3) polarity   +1 (field up, "write") / -1 (field down, "reset")
%        u(4) I_lim      peak-current cutoff for the pulse [A] (0 = none)
%        u(5) I_trim     trim-coil current command [A]
%        u(6) f_probe    probe-tone frequency for the detector [Hz]
%
%   ------------------------------------------------------------------
%   BASELINE STRATEGY (the vendor procedure, closed around f_meas):
%     1. RESET : natural pulse at -V_reset -> magnet at its lowest field
%                (a known state; this is what the device LUT assumes).
%     2. WRITE : natural pulse at +V_k, first guess V_k = LUT(f_target).
%     3. MEASURE f. If it is off by more than write_tol:
%          too low  -> fire a bigger write; no reset needed, because a
%                      larger pulse wipes out the smaller one (Preisach)
%          too high -> reset, then write a smaller V_k
%        V_k is found by secant steps with a bisection fallback.
%     4. TRACK : the magnet holds with zero power; a PI loop on the trim
%                coil removes the residual error and drift. If the trim
%                runs out of range -> rewrite.
%   The LUT is refined online after every successful write.
%   ------------------------------------------------------------------

% mode ids
RESET_CHARGE = 1;  PULSE_WAIT = 2;  WRITE_CHARGE = 3;  MEASURE = 4;  TRACK = 5;

persistent mode next_mode timer f_act V_k V_lo V_hi has_lo has_hi ...
           V_prev f_prev has_prev iter V_max_since_reset f_acc n_acc ...
           integ sat_timer cal_V dVdf0 dVdf fire_out V0 pol
if isempty(mode)
    cal_V = P.ctrl.cal_V(:);
    dVdf0 = (cal_V(end) - cal_V(1)) / (P.ctrl.cal_f(end) - P.ctrl.cal_f(1));
    f_act = f_target;
    [mode, timer, iter, has_lo, has_hi, has_prev, integ, V_k, dVdf] = ...
        start_write(f_act, P, cal_V, dVdf0, RESET_CHARGE);
    next_mode = WRITE_CHARGE;
    V_lo = 0;  V_hi = 0;  V_prev = 0;  f_prev = 0;
    V_max_since_reset = 0;  f_acc = 0;  n_acc = 0;  sat_timer = 0;
    fire_out = 0;  V0 = 0;  pol = 1;
end

Tc = P.sim.Tc;
timer = timer + Tc;
fire_out = 0;
trim_range = P.trim.Imax*P.trim.kB*P.yig.gamma;   % [Hz]

f_meas = y(3);  busy = y(6);  ready = y(7);

% ---------- new target? ----------------------------------------------
if abs(f_target - f_act) > P.ctrl.retarget_tol
    % trim needed after the move = what it supplies now + the step
    trim_needed = integ*P.trim.kB*P.yig.gamma + (f_target - f_act);
    small_move = (mode == TRACK) && (abs(trim_needed) < 0.5*trim_range);
    f_act = f_target;
    if ~small_move
        [mode, timer, iter, has_lo, has_hi, has_prev, integ, V_k, dVdf] = ...
            start_write(f_act, P, cal_V, dVdf0, RESET_CHARGE);
    end
end

switch mode
    case RESET_CHARGE  % ---- charge to V_reset, fire a - pulse ---------
        V0 = P.ctrl.V_reset;  pol = -1;  integ = 0;
        if timer > 3*Tc && ready > 0.5 && busy < 0.5
            fire_out = 1;  mode = PULSE_WAIT;  next_mode = WRITE_CHARGE;
            timer = 0;  V_max_since_reset = 0;
        end

    case PULSE_WAIT    % ---- wait for the pulse to finish ---------------
        if timer > 3*Tc && busy < 0.5
            mode = next_mode;  timer = 0;  f_acc = 0;  n_acc = 0;
        end

    case WRITE_CHARGE  % ---- charge to V_k, fire a + pulse -------------
        V0 = V_k;  pol = 1;  integ = 0;
        if timer > 3*Tc && ready > 0.5 && busy < 0.5
            fire_out = 1;  mode = PULSE_WAIT;  next_mode = MEASURE;
            timer = 0;  V_max_since_reset = max(V_max_since_reset, V_k);
        end

    case MEASURE       % ---- let eddy currents settle, average f -------
        V0 = P.ctrl.V_idle;
        if timer > P.ctrl.t_settle
            f_acc = f_acc + f_meas;  n_acc = n_acc + 1;
            if n_acc >= P.ctrl.n_avg
                f_k = f_acc / n_acc;
                err = f_k - f_act;
                iter = iter + 1;
                % ---- local slope dV/df from the last two writes ----
                if has_prev > 0.5 && abs(f_k - f_prev) > 1e6
                    s = (V_k - V_prev)/(f_k - f_prev);
                    if s > 0.2*dVdf0 && s < 5*dVdf0
                        dVdf = s;
                    end
                end
                V_prev = V_k;  f_prev = f_k;  has_prev = 1;
                if abs(err) < P.ctrl.write_tol || iter >= P.ctrl.max_iter
                    % ---- done: learn, then hold with the trim coil ----
                    V_at_target = V_k + (f_act - f_k)*dVdf;
                    cal_V = cal_update(P.ctrl.cal_f(:), cal_V, f_act, V_at_target);
                    mode = TRACK;  timer = 0;  integ = 0;  sat_timer = 0;
                else
                    % ---- bracket (f increases with V) and next guess ----
                    if err > 0
                        V_hi = V_k;  has_hi = 1;
                    else
                        V_lo = V_k;  has_lo = 1;
                    end
                    V_new = V_k + (f_act - f_k)*dVdf;          % secant step
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
                    end
                end
            end
        end

    otherwise          % ---- TRACK: zero-power hold + trim PI ----------
        V0 = P.ctrl.V_idle;  pol = 1;
        e = f_act - f_meas;
        integ = integ + P.ctrl.Ki_trim*e*Tc;
        integ = min(max(integ, -P.trim.Imax), P.trim.Imax);
        if abs(integ) > P.ctrl.trim_frac*P.trim.Imax
            sat_timer = sat_timer + Tc;
        else
            sat_timer = 0;
        end
        if sat_timer > P.ctrl.t_trim_sat           % drifted out of trim range
            [mode, timer, iter, has_lo, has_hi, has_prev, integ, V_k, dVdf] = ...
                start_write(f_act, P, cal_V, dVdf0, RESET_CHARGE);
            sat_timer = 0;
        end
end

if mode == TRACK
    I_trim = min(max(integ + P.ctrl.Kp_trim*(f_act - f_meas), -P.trim.Imax), P.trim.Imax);
else
    I_trim = 0;
end

u = [V0; fire_out; pol; 0; I_trim; f_act];   % I_lim = 0: natural pulses only
end

%% ======================================================================
function [mode, timer, iter, has_lo, has_hi, has_prev, integ, V_k, dVdf] = ...
         start_write(f_t, P, cal_V, dVdf0, RESET_CHARGE)
mode = RESET_CHARGE;  timer = 0;  iter = 0;
has_lo = 0;  has_hi = 0;  has_prev = 0;  integ = 0;
V_k  = min(max(cal_lookup(P.ctrl.cal_f(:), cal_V, f_t), 0), P.ctrl.V_write_max);
dVdf = local_slope(P.ctrl.cal_f(:), cal_V, f_t, dVdf0);
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
