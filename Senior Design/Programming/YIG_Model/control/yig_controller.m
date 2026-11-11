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
%        u(3) polarity   +1 / -1
%        u(4) I_lim      peak-current cutoff for the pulse [A] (0 = none)
%        u(5) I_trim     trim-coil current command [A]
%        u(6) f_probe    probe-tone frequency for the detector [Hz]
%
%   ------------------------------------------------------------------
%   BASELINE STRATEGY (replace freely):
%     1. SATURATE  : big + pulse -> magnet at positive remanence (known state)
%     2. KNOCK-DOWN: - pulse with peak current I_k lowers the remanence.
%                    f(I_k) is monotone, so search I_k with a bracketed
%                    secant.  Larger knocks don't need a re-saturate
%                    (Preisach wipe-out); smaller ones do.
%     3. TRACK     : magnet holds with zero power; PI on the trim coil
%                    removes the residual error and temperature drift.
%                    If the trim runs out of range -> rewrite.
%   A cal table (f -> I_k) is refined after every successful write.
%   ------------------------------------------------------------------

% mode ids
SAT_CHARGE = 1;  PULSE_WAIT = 2;  KNOCK_CHARGE = 3;  MEASURE = 4;  TRACK = 5;

persistent mode next_mode timer f_act I_k I_lo f_lo I_hi f_hi has_lo has_hi ...
           iter I_max_since_sat f_acc n_acc integ sat_timer cal_I dIdf ...
           fire_out V0 pol Ilim
if isempty(mode)
    mode = SAT_CHARGE;  next_mode = KNOCK_CHARGE;  timer = 0;
    f_act = f_target;
    cal_I = P.ctrl.cal_I(:);
    dIdf  = (cal_I(end) - cal_I(1)) / (P.ctrl.cal_f(end) - P.ctrl.cal_f(1));
    I_k = 0; I_lo = 0; f_lo = 0; I_hi = 0; f_hi = 0; has_lo = 0; has_hi = 0;
    iter = 0; I_max_since_sat = 0; f_acc = 0; n_acc = 0;
    integ = 0; sat_timer = 0;
    fire_out = 0; V0 = 0; pol = 1; Ilim = 0;
    I_k = cal_lookup(P.ctrl.cal_f(:), cal_I, f_act);
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
        [mode, timer, iter, has_lo, has_hi, integ, I_k] = ...
            start_write(f_act, P, cal_I, SAT_CHARGE);
    end
end

switch mode
    case SAT_CHARGE    % ---- charge to V_sat, then fire + pulse --------
        V0 = P.ctrl.V_sat;  pol = 1;  Ilim = 0;  integ = 0;
        if timer > 3*Tc && ready > 0.5 && busy < 0.5
            fire_out = 1;  mode = PULSE_WAIT;  next_mode = KNOCK_CHARGE;
            timer = 0;  I_max_since_sat = 0;
        end

    case PULSE_WAIT    % ---- wait for the pulse to finish ---------------
        if timer > 3*Tc && busy < 0.5
            mode = next_mode;  timer = 0;  f_acc = 0;  n_acc = 0;
        end

    case KNOCK_CHARGE  % ---- charge to V_knock, fire - pulse at I_k ----
        V0 = P.ctrl.V_knock;  pol = -1;  Ilim = I_k;  integ = 0;
        if timer > 3*Tc && ready > 0.5 && busy < 0.5
            fire_out = 1;  mode = PULSE_WAIT;  next_mode = MEASURE;
            timer = 0;  I_max_since_sat = max(I_max_since_sat, I_k);
        end

    case MEASURE       % ---- let eddy currents settle, average f -------
        V0 = P.ctrl.V_idle;
        if timer > P.ctrl.t_settle
            f_acc = f_acc + f_meas;  n_acc = n_acc + 1;
            if n_acc >= P.ctrl.n_avg
                f_k = f_acc / n_acc;
                err = f_k - f_act;
                iter = iter + 1;
                if abs(err) < P.ctrl.write_tol || iter >= P.ctrl.max_iter
                    % ---- done: learn, then hold with the trim coil ----
                    I_at_target = I_k + (f_act - f_k)*dIdf;
                    cal_I = cal_update(P.ctrl.cal_f(:), cal_I, f_act, I_at_target);
                    mode = TRACK;  timer = 0;  integ = 0;  sat_timer = 0;
                else
                    % ---- update bracket (f decreases as I_k grows) ----
                    if err > 0
                        if has_hi > 0.5 && I_hi ~= I_k
                            dIdf = (I_hi - I_k)/(f_hi - f_k);
                        elseif has_lo > 0.5 && I_lo ~= I_k
                            dIdf = (I_lo - I_k)/(f_lo - f_k);
                        end
                        I_lo = I_k;  f_lo = f_k;  has_lo = 1;
                    else
                        if has_lo > 0.5 && I_lo ~= I_k
                            dIdf = (I_lo - I_k)/(f_lo - f_k);
                        elseif has_hi > 0.5 && I_hi ~= I_k
                            dIdf = (I_hi - I_k)/(f_hi - f_k);
                        end
                        I_hi = I_k;  f_hi = f_k;  has_hi = 1;
                    end
                    if ~(dIdf < 0) || ~isfinite(dIdf)     % keep it sane
                        dIdf = (P.ctrl.cal_I(end) - P.ctrl.cal_I(1)) / ...
                               (P.ctrl.cal_f(end) - P.ctrl.cal_f(1));
                    end
                    I_new = I_k + (f_act - f_k)*dIdf;          % Newton/secant step
                    if has_lo > 0.5 && has_hi > 0.5
                        w = I_hi - I_lo;
                        if I_new <= I_lo + 0.02*w || I_new >= I_hi - 0.02*w
                            I_new = 0.5*(I_lo + I_hi);         % bisection fallback
                        end
                    end
                    I_k = min(max(I_new, 0), P.ctrl.I_knock_max);
                    timer = 0;
                    if I_k > I_max_since_sat
                        mode = KNOCK_CHARGE;   % deeper knock: no reset needed
                    else
                        mode = SAT_CHARGE;     % shallower: must re-saturate
                    end
                end
            end
        end

    otherwise          % ---- TRACK: zero-power hold + trim PI ----------
        V0 = P.ctrl.V_idle;  pol = 1;  Ilim = 0;
        e = f_act - f_meas;
        integ = integ + P.ctrl.Ki_trim*e*Tc;
        integ = min(max(integ, -P.trim.Imax), P.trim.Imax);
        if abs(integ) > P.ctrl.trim_frac*P.trim.Imax
            sat_timer = sat_timer + Tc;
        else
            sat_timer = 0;
        end
        if sat_timer > P.ctrl.t_trim_sat           % drifted out of trim range
            [mode, timer, iter, has_lo, has_hi, integ, I_k] = ...
                start_write(f_act, P, cal_I, SAT_CHARGE);
            sat_timer = 0;
        end
end

if mode == TRACK
    I_trim = min(max(integ + P.ctrl.Kp_trim*(f_act - f_meas), -P.trim.Imax), P.trim.Imax);
else
    I_trim = 0;
end

u = [V0; fire_out; pol; Ilim; I_trim; f_act];
end

%% ======================================================================
function [mode, timer, iter, has_lo, has_hi, integ, I_k] = start_write(f_t, P, cal_I, SAT_CHARGE)
mode = SAT_CHARGE;  timer = 0;  iter = 0;  has_lo = 0;  has_hi = 0;  integ = 0;
I_k = min(max(cal_lookup(P.ctrl.cal_f(:), cal_I, f_t), 0), P.ctrl.I_knock_max);
end

function I = cal_lookup(cf, cI, f)
% linear interpolation with linear extrapolation at the ends
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
t = (f - cf(j)) / (cf(j+1) - cf(j));
I = (1 - t)*cI(j) + t*cI(j+1);
end

function cI = cal_update(cf, cI, f, I_true)
% nudge the two neighbouring nodes so the table hits (f, I_true) exactly
n = numel(cf);
fc = min(max(f, cf(1)), cf(n));
j = 1;
while j < n - 1 && fc > cf(j+1)
    j = j + 1;
end
t = (fc - cf(j)) / (cf(j+1) - cf(j));
w1 = 1 - t;  w2 = t;
delta = I_true - cal_lookup(cf, cI, fc);
s = w1^2 + w2^2;
cI(j)   = cI(j)   + delta*w1/s;
cI(j+1) = cI(j+1) + delta*w2/s;
end
