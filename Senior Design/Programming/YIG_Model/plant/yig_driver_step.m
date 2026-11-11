function drv = yig_driver_step(u, P)
%YIG_DRIVER_STEP  Charger + storage cap + H-bridge + main coil + trim coil.
%   One plant step (P.sim.Ts).  Codegen-compatible (used in Simulink).
%
%   u   : controller command (6x1)  -- see yig_controller.m
%         u(1) V0_set   charge target [V]
%         u(2) fire     rising edge fires a pulse (ignored while busy)
%         u(3) polarity +1 / -1 (current direction in the main coil)
%         u(4) I_lim    peak-current cutoff [A]; 0 = natural peak (di/dt = 0)
%         u(5) I_trim   trim-coil current setpoint [A]
%         u(6) f_probe  (not used here)
%
%   drv : [I_coil; Vc; busy; ready; I_trim]
%
%   Phases:  0 idle/charging   1 bridge ON (cap -> coil)
%            2 bridge OFF, diode recovery (coil -> cap)

persistent Vc ic phase pol Ilim fire_prev Itrim
if isempty(Vc)
    Vc = P.drv.Vc0;  ic = 0;  phase = 0;  pol = 1;  Ilim = 0;
    fire_prev = 0;   Itrim = 0;
end

Ts = P.sim.Ts;  L = P.drv.L;  R = P.drv.R;  C = P.drv.C;

V0_set = min(max(u(1), 0), P.drv.Vmax);
fire   = double(u(2) > 0.5);   % double, so it can live in a persistent
Itrim_set = min(max(u(5), -P.trim.Imax), P.trim.Imax);

I_out = ic;
switch phase
    case 0  % ---------------- idle: charger regulates Vc ---------------
        dV = Ts*P.drv.I_chg/C;
        if Vc < V0_set
            Vc = min(Vc + dV, V0_set);
        elseif Vc > V0_set
            Vc = max(Vc - Ts*P.drv.I_dump/C, V0_set);
        end
        if fire > 0.5 && fire_prev < 0.5
            phase = 1;
            if u(3) < 0, pol = -1; else, pol = 1; end
            if u(4) > 0
                Ilim = u(4) + P.drv.Ilim_err_sd*randn();
            else
                Ilim = 0;
            end
        end

    otherwise
        % Pulse in progress. The coil time constants are only a few us
        % (L/R = 2.3 us with the measured coil), so integrate the pulse
        % in nsub sub-steps of Ts/nsub.
        % The magnet only sees one current per Ts, and its final state is
        % set by the current EXTREMA (Preisach), so report the largest
        % current reached inside the step, not the end-of-step value.
        dt = Ts/P.drv.nsub;
        ipk = ic;
        for s = 1:P.drv.nsub
            if phase == 1   % ---------- bridge ON: Vc drives the coil ----
                ic_new = ic + dt*(Vc - R*ic)/L;
                Vc_new = Vc - dt*ic_new/C;
                if Ilim > 0 && ic_new >= Ilim
                    % comparator trips: clamp exactly at the threshold
                    ic = Ilim;  Vc = Vc - dt*Ilim/C;  phase = 2;
                elseif ic_new <= ic || Vc_new <= 0
                    % natural peak: current stops rising (di/dt <= 0,
                    % i.e. Vc <= R*ic).  Underdamped this is ~Vc = 0;
                    % overdamped (R > 2*sqrt(L/C)) Vc never reaches 0.
                    Vc = max(Vc_new, 0);  phase = 2;
                else
                    ic = ic_new;  Vc = Vc_new;
                end
            elseif phase == 2   % ---- bridge OFF: diodes return energy --
                ic_new = ic + dt*(-Vc - R*ic - 2*P.drv.Vd)/L;
                if ic_new <= 0
                    ic = 0;  phase = 0;
                else
                    ic = ic_new;  Vc = Vc + dt*ic/C;
                end
            end
            ipk = max(ipk, ic);
        end
        I_out = ipk;
end
fire_prev = fire;

% trim coil: first-order current loop
Itrim = Itrim + Ts/P.trim.tau*(Itrim_set - Itrim);

busy  = double(phase ~= 0);
ready = double(phase == 0 && abs(Vc - V0_set) <= P.drv.ready_tol);
drv = [pol*I_out; Vc; busy; ready; Itrim];
end
