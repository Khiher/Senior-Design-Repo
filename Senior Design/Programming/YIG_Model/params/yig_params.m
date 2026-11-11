function P = yig_params()
%YIG_PARAMS  All physical + simulation parameters for the YIG tuning model.
%
%   P = yig_params() returns one struct that every plant block and the
%   controller block receive as the Simulink parameter "YIG".
%
%   *** Every number here is a PLACEHOLDER chosen to be physically
%   *** plausible for an AlNiCo-5 latched YIG filter. Replace them with
%   *** measured values from your coil, magnet, and filter as you get them.
%
%   Rules (so the struct works as a MATLAB Function block parameter):
%     - numeric / logical fields only (no strings, no cell arrays)
%     - do not change array SIZES between builds without re-running
%       yig_setup (sizes are compiled in)

%% ---------------- Simulation timing ----------------------------------
P.sim.Ts    = 1e-6;     % plant step [s]   (pulses are ~10 us; the driver sub-steps them, see P.drv.nsub)
P.sim.Tc    = 1e-4;     % controller step [s] (10 kHz control loop)
P.sim.Tstop = 0.40;     % default stop time [s]
P.sim.log_decim = 10;   % log every Nth plant sample

P.T0 = 25;              % reference temperature [degC]

%% ---------------- Pulse driver: charger + cap + H-bridge + coil -------
%  Cap discharges into the main coil through the H-bridge. The bridge
%  turns OFF when (a) the coil current reaches its natural peak (di/dt = 0,
%  i.e. Vc = R*i; ~Vc = 0 if underdamped) or (b) the coil current reaches
%  the commanded peak I_lim (peak-current mode). After turn-off the
%  current freewheels through the body diodes back INTO the cap (energy
%  recovery) until it reaches zero.
%  The measured coil below is OVERDAMPED (R = 3 ohm > 2*sqrt(L/C) = 1.17 ohm):
%  a 100 V natural pulse peaks at ~30 A after ~8 us, with Vc still ~90 V.
P.drv.L       = 6.812e-6;   % main coil inductance [H]
P.drv.R       = 3;    % coil + switch + ESR resistance [ohm]
P.drv.C       = 20e-6;  % storage capacitor [F]
P.drv.Vd      = 0.8;    % body-diode drop during recovery [V] (x2 diodes)
P.drv.Vmax    = 100;    % max allowed cap voltage [V]
P.drv.I_chg   = 2.0;    % constant-current charger [A]
P.drv.I_dump  = 2.0;    % active bleed current when V0_set < Vc [A]
P.drv.ready_tol = 0.5;  % |Vc - V0_set| below this => "ready" [V]
P.drv.Ilim_err_sd = 1e-3; % peak-current comparator repeatability (1 sigma) [A]
P.drv.Vc0     = 0;      % initial cap voltage [V]
P.drv.nsub    = 50;     % pulse integration sub-steps per Ts (Ts/nsub = 20 ns)

%% ---------------- Trim (FM) coil ---------------------------------------
P.trim.Imax = 1.0;      % trim driver limit [A]
P.trim.tau  = 20e-6;    % trim current-loop time constant [s]
P.trim.kB   = 2e-3;     % gap field per trim amp [T/A]  (~56 MHz/A)

%% ---------------- AlNiCo magnet (Preisach hysteresis) ------------------
%  Hysteron distribution: Gaussian in coercive field hc and interaction
%  field hu  (alpha = hu + hc, beta = hu - hc). Output m is normalised
%  magnetisation in [-1, 1]. Self-demagnetisation of the magnetic circuit
%  (the gap) is H_eff = H_coil - D*m, solved implicitly every step.
P.mag.kH      = 15e3;   % applied field per coil amp  N/l_eff [A/m per A]
P.mag.D       = 25e3;   % demag field at m = 1  (Nd*Ms of the circuit) [A/m]
P.mag.hc_mean = 50e3;   % mean intrinsic coercivity (AlNiCo 5 ~ 50 kA/m) [A/m]
P.mag.hc_sd   = 12e3;   % coercivity spread [A/m]
P.mag.hu_sd   = 8e3;    % interaction-field spread [A/m]
P.mag.init    = 0;      % initial state: -1 neg. saturated, 0 demagnetised, +1 pos. saturated
P.mag.eps_rev = 2;      % reversal dead-band (numerical) [A/m]

% Build the Everett function table E(alpha,beta) (normalised so that
% E(Hm,-Hm) = 1).  Bilinear interpolation of this table gives a smooth,
% continuous hysteresis model with correct minor loops / wipe-out.
Hm   = P.mag.hc_mean + 4*P.mag.hc_sd + 4*P.mag.hu_sd;   % support limit
Ng   = 241;
h    = linspace(-Hm, Hm, Ng);
[A, B] = ndgrid(h, h);                 % A = alpha (rows), B = beta (cols)
hc   = (A - B)/2;   hu = (A + B)/2;
mu   = exp(-0.5*((hc - P.mag.hc_mean)/P.mag.hc_sd).^2) .* ...
       exp(-0.5*(hu/P.mag.hu_sd).^2);
mu(A < B) = 0;
% E(i,j) = sum_{a<=i} sum_{b>=j} mu(a,b)
E = cumsum(mu, 1);
E = fliplr(cumsum(fliplr(E), 2));
E = E / E(end, 1);
P.mag.Hm   = Hm;
P.mag.Ng   = Ng;
P.mag.dH   = h(2) - h(1);
P.mag.E    = E;

%% ---------------- Gap field / magnetic circuit ------------------------
P.gap.Bmax   = 0.70;    % gap flux density at m = 1 [T]
P.gap.tc     = -2e-4;   % reversible tempco of AlNiCo Br [1/degC] (-0.02 %/C)
P.gap.k_main = 4e-3;    % direct gap field from main-coil current [T/A] (only while pulsing)
P.gap.tau_eddy = 100e-6;% eddy-current lag in pole pieces [s]

%% ---------------- YIG resonator / filter ------------------------------
P.yig.gamma   = 28.0e9; % gyromagnetic ratio [Hz/T]
P.yig.f_anis  = 0;      % anisotropy / shape offset at T0 [Hz]
P.yig.tc_anis = 0;      % its tempco [Hz/degC]
P.yig.BW      = 30e6;   % 3 dB bandwidth [Hz]
P.yig.nstage  = 2;      % number of resonator stages (sets skirt steepness)
P.yig.IL_dB   = 4;      % insertion loss at centre [dB]
P.yig.Pin_dBm = -10;    % probe tone power into filter [dBm]
P.yig.floor_dBm = -70;  % detector noise floor [dBm]

%% ---------------- Sensors ---------------------------------------------
P.sens.I_lsb   = 5e-3;  % coil-current ADC LSB [A]
P.sens.I_sd    = 5e-3;  % coil-current noise [A rms]
P.sens.V_lsb   = 0.1;   % cap-voltage ADC LSB [V]
P.sens.V_sd    = 0.1;   % cap-voltage noise [V rms]
P.sens.f_sd    = 0.5e6; % centre-frequency measurement noise [Hz rms]
P.sens.P_sd    = 0.1;   % detector noise [dB rms]
P.sens.T_sd    = 0.02;  % temperature-sensor noise [degC rms]

%% ---------------- Baseline controller tuning --------------------------
%  (Used only by control/yig_controller.m -- add whatever your own
%   controller needs here.)
P.ctrl.V_sat     = 100;   % bus voltage for the saturating (reset) pulse [V] (= drv.Vmax; natural peak ~30 A)
P.ctrl.V_knock   = 20;    % bus voltage for the knock-down pulse [V] (natural peak ~6 A > I_knock_max;
                          %  keeps di/dt ~3 A/us so comparator delay barely overshoots I_lim)
P.ctrl.I_knock_max = 3.4; % largest knock-down peak current (keeps m > 0) [A]
P.ctrl.V_idle    = 0;     % cap voltage while holding (0 = fully discharged, lowest energy) [V]
P.ctrl.write_tol = 15e6;  % accept a write when |f - f_target| < this [Hz]
P.ctrl.max_iter  = 10;    % max knock-down iterations per write
P.ctrl.t_settle  = 1.0e-3;% wait after a pulse before measuring [s]
P.ctrl.n_avg     = 10;    % f samples averaged per measurement
P.ctrl.retarget_tol = 1e6;% target change that counts as a new target [Hz]
P.ctrl.Ki_trim   = 2.0e-5;% trim integral gain [A/(Hz*s)]
P.ctrl.Kp_trim   = 2.0e-9;% trim proportional gain [A/Hz]
P.ctrl.trim_frac = 0.9;   % rewrite magnet if |I_trim| > frac*Imax ...
P.ctrl.t_trim_sat = 5e-3; % ... for longer than this [s]
% Initial guess table: knock-down peak current vs target frequency.
% Refined online by the controller; replace with a measured calibration.
P.ctrl.cal_f = linspace(2e9, 18e9, 9);   % [Hz]  (must be increasing)
P.ctrl.cal_I = linspace(3.2, 0.2, 9);    % [A]   deliberately crude linear guess
end
