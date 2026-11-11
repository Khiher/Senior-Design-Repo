function P = yig_params(dev)
%YIG_PARAMS  All physical + simulation parameters for the YIG tuning model.
%
%   P = yig_params(dev) returns one struct that every plant block and the
%   controller block receive as the Simulink parameter "YIG".
%     dev : device LUT struct from devices/ (e.g. pcb1_yig12_20260223()).
%           Its LUT becomes the controller's initial voltage table, and
%           yig_fit_device() then fits the magnet so the model reproduces it.
%
%   Values marked (measured) or (vendor) come from real hardware / the YIG
%   vendor. Everything else is a PLACEHOLDER; replace it as you measure.
%
%   Rules (so the struct works as a MATLAB Function block parameter):
%     - numeric / logical fields only (no strings, no cell arrays)
%     - do not change array SIZES between builds without re-running
%       yig_setup (sizes are compiled in)

if nargin < 1, dev = pcb1_yig12_20260223(); end

%% ---------------- Simulation timing ----------------------------------
P.sim.Ts    = 1e-6;     % plant step [s]   (pulses are ~10 us; the driver sub-steps them, see P.drv.nsub)
P.sim.Tc    = 20e-6;    % controller step [s] (50 kHz control loop; needed for the < 1 ms response spec)
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
%  a natural pulse peaks at ~0.30 A per volt after ~8 us, with Vc still ~0.9*V0.
P.drv.L       = 6.812e-6;   % main coil inductance [H]            (measured)
P.drv.R       = 3;    % coil + switch + ESR resistance [ohm]      (measured)
P.drv.C       = 20e-6;  % storage capacitor [F]                   (measured)
P.drv.Vd      = 0.8;    % body-diode drop during recovery [V] (x2 diodes)
P.drv.Vmax    = 100;    % max allowed cap voltage [V]             (measured)
P.drv.I_chg   = 10;     % constant-current charger [A]  (0 -> 50 V in 100 us; placeholder)
P.drv.I_dump  = 10;     % active bleed current when V0_set < Vc [A]  (placeholder)
P.drv.ready_tol = 0.5;  % |Vc - V0_set| below this => "ready" [V]
P.drv.Vset_err_sd = 5e-3; % charge-voltage error at the moment of firing (1 sigma) [V]
                          %  The LUT slope is ~0.5 GHz/V, so 5 mV is ~3 MHz per write.
                          %  Placeholder, but effectively a REQUIREMENT on the charger:
                          %  much worse and the magnet alone can't land within 10 MHz.
P.drv.Ilim_err_sd = 1e-3; % peak-current comparator repeatability (1 sigma) [A]
P.drv.Vc0     = 0;      % initial cap voltage [V]
P.drv.nsub    = 50;     % pulse integration sub-steps per Ts (Ts/nsub = 20 ns)

%% ---------------- Trim (FM) coil ---------------------------------------
P.trim.Imax = 1.0;      % trim driver limit [A]
P.trim.tau  = 20e-6;    % trim current-loop time constant [s]
P.trim.kB   = 2e-3;     % gap field per trim amp [T/A]  (20 G/A = 56 MHz/A)

%% ---------------- AlNiCo magnet (Preisach hysteresis) ------------------
%  Hysteron distribution: Gaussian in coercive field hc and interaction
%  field hu  (alpha = hu + hc, beta = hu - hc). Output m is normalised
%  magnetisation in [-1, 1]. Self-demagnetisation of the magnetic circuit
%  (the gap) is H_eff = H_coil - D*m, solved implicitly every step.
%  hc_mean and hc_sd are FITTED to the device LUT by yig_fit_device().
P.mag.kH      = 15e3;   % applied field per coil amp  N/l_eff [A/m per A]
P.mag.D       = 25e3;   % demag field at m = 1  (Nd*Ms of the circuit) [A/m]
P.mag.hc_mean = 110e3;  % mean coercivity [A/m]          (fitted)
P.mag.hc_sd   = 20e3;   % coercivity spread [A/m]        (fitted)
P.mag.hu_sd   = 8e3;    % interaction-field spread [A/m]
P.mag.init    = 0;      % initial state: -1 neg. saturated, 0 demagnetised, +1 pos. saturated
P.mag.eps_rev = 2;      % reversal dead-band (numerical) [A/m]
P = yig_build_everett(P);

%% ---------------- Gap field / magnetic circuit ------------------------
%  B_gap = (B_pass + Bmax*m)*(1 + tc*(T - T0)) + coils + stray field.
%  B_pass is the passive (permanent-magnet) bias; Bmax*m is the AlNiCo
%  part. Resetting (m -> -1) gives the LOWEST field, as in the hardware.
P.gap.B_pass = 0.26;    % passive bias field [T]                  (fitted)
P.gap.Bmax   = 0.23;    % AlNiCo contribution at m = 1 [T]        (fitted)
P.gap.tc     = -2e-4;   % reversible tempco of the magnet field [1/degC] (-0.02 %/C)
P.gap.k_main = 4e-3;    % direct gap field from main-coil current [T/A] (only while pulsing)
P.gap.tau_eddy = 100e-6;% eddy-current lag in pole pieces [s]  (placeholder; sets the
                        %  settle time after a write, so it dominates response time)

%% ---------------- YIG resonator / filter ------------------------------
%  Centre frequency (vendor): fc [MHz] = 2.8 * B [G] + 2217.
%  The passband shape below is a rough stand-in. The vendor says a real
%  YIG response needs FEM (e.g. HFSS); only the detector reading y(4) uses
%  it, the controller tunes on the centre frequency.
P.yig.gamma   = 28.0e9; % 2.8 MHz/G = 28 GHz/T [Hz/T]            (vendor)
P.yig.f_anis  = 2.217e9;% frequency offset at B = 0 [Hz]         (vendor)
P.yig.tc_anis = 0;      % its tempco [Hz/degC]
P.yig.BW      = 30e6;   % 3 dB bandwidth [Hz]                     (placeholder shape)
P.yig.nstage  = 2;      % number of resonator stages              (placeholder shape)
P.yig.IL_dB   = 4;      % insertion loss at centre [dB]
P.yig.Pin_dBm = -10;    % probe tone power into filter [dBm]
P.yig.floor_dBm = -70;  % detector noise floor [dBm]

%% ---------------- Sensors ---------------------------------------------
P.sens.I_lsb   = 5e-3;  % coil-current ADC LSB [A]
P.sens.I_sd    = 5e-3;  % coil-current noise [A rms]
P.sens.V_lsb   = 0.1;   % cap-voltage ADC LSB [V]
P.sens.V_sd    = 0.1;   % cap-voltage noise [V rms]
P.sens.P_sd    = 0.1;   % detector noise [dB rms]
P.sens.T_sd    = 0.02;  % temperature-sensor noise [degC rms]

%% ---------------- Hall sensor (frequency feedback) --------------------
%  Measures the field near the YIG sphere; the controller turns it into a
%  frequency with a bench calibration (yig_calibrate_hall). All placeholders,
%  roughly a good linear Hall IC with a 16-bit ADC. 1 G = 1e-4 T = 2.8 MHz.
P.hall.k         = 0.8;     % fraction of the gap field at the sensor location
P.hall.gain_err  = 0.01;    % sensitivity error at T0 (removed by calibration)
P.hall.offset    = 2e-3;    % offset at T0 [T] (20 G; removed by calibration)
P.hall.tc_gain   = -400e-6; % true sensitivity tempco [1/degC]
P.hall.tc_offset = 5e-6;    % offset drift [T/degC] (0.05 G/degC; not compensated)
P.hall.noise_sd  = 50e-6;   % noise per sample [T rms] (0.5 G, ~1.4 MHz)
P.hall.tau       = 8e-6;    % sensor bandwidth as a time constant [s] (~20 kHz)
P.hall.FS        = 0.5;     % ADC full scale +/- [T]
P.hall.bits      = 16;      % ADC resolution (LSB = 15 uT = 0.15 G = 0.4 MHz)

%% ---------------- Power (for the < 1 mW standby spec) -----------------
%  Placeholders. Standby = magnet holding on its own, trim off, MCU asleep,
%  Hall powered only during periodic wake-up checks.
P.pwr.R_trim  = 1.0;      % trim-coil resistance [ohm]
P.pwr.P_sleep = 0.2e-3;   % MCU stop mode + regulators + leakage [W]
P.pwr.P_hall  = 20e-3;    % Hall sensor + ADC while powered [W] (3.3 V x 6 mA)
P.pwr.P_run   = 50e-3;    % MCU running + Hall, while actively tuning [W] (reported only)

%% ---------------- Device LUT (vendor calibration) ---------------------
%  Write voltage vs centre frequency after a reset pulse of V_reset.
%  Polarity in the model: reset = -1 (field down), write = +1 (field up).
P.lut.f       = dev.f(:)';   % [Hz]  (increasing)
P.lut.V       = dev.V(:)';   % [V]
P.lut.V_reset = dev.V_reset; % [V]

%% ---------------- Baseline controller tuning --------------------------
%  (Used only by control/yig_controller.m -- add whatever your own
%   controller needs here.)
P.ctrl.V_reset   = dev.V_reset; % reset pulse voltage, negative polarity [V]
P.ctrl.V_write_max = dev.V_reset; % never write harder than the reset can erase [V]
P.ctrl.V_idle    = 0;     % cap voltage while holding (0 = fully discharged, lowest energy) [V]
P.ctrl.spec_tol  = 10e6;  % accuracy spec [Hz]
P.ctrl.write_tol = 40e6;  % accept a write when |f_magnet - f_target| < this; the trim coil does the rest [Hz]
P.ctrl.write_bias = 20e6; % aim writes this far LOW: the magnet can only be raised without a reset,
                          %  so a write is accepted only if it lands between -write_tol and
                          %  +standby_enter_tol of the target; the trim covers the rest [Hz]
P.ctrl.max_iter  = 10;    % max write pulses per target
P.ctrl.t_settle  = 7.5*P.gap.tau_eddy; % wait after a pulse before measuring [s] (eddy decays to 0.05%)
P.ctrl.n_avg     = 5;     % Hall samples averaged per measurement
P.ctrl.retarget_tol = 1e6;% target change that counts as a new target [Hz]
P.ctrl.Ki_trim   = 5.0e-5;% trim integral gain [A/(Hz*s)]
P.ctrl.Kp_trim   = 5.0e-9;% trim proportional gain [A/Hz]
P.ctrl.trim_frac = 0.9;   % rewrite magnet if |I_trim| > frac*Imax ...
P.ctrl.t_trim_sat = 0.5e-3; % ... for longer than this [s]
% standby: after holding t_standby, nudge the magnet until the trim is ~0,
% then switch trim + Hall off and wake every t_wake to check for drift
P.ctrl.t_standby = 5e-3;  % hold time before going to standby [s]
P.ctrl.standby_enter_tol = 6e6; % trim may supply at most this when entering standby [Hz]
P.ctrl.standby_wake_tol  = 8e6; % on a wake check, re-tune if the error exceeds this [Hz]
P.ctrl.max_nudge = 8;     % refine pulses allowed per standby entry
P.ctrl.nudge_gain = 0.5;  % size a nudge for this fraction of the error (the local
                          %  slope is uncertain; undershoot is cheap, overshoot needs a reset)
P.ctrl.t_wake    = 20e-3; % wake-up period in standby [s] (placeholder; real drift is slow)
P.ctrl.t_hall_on = 50e-6; % Hall power-up settle before readings are trusted [s]
% Hall -> frequency conversion (filled in by yig_calibrate_hall)
P.ctrl.hall_tc_comp = -350e-6; % gain tempco the firmware assumes (datasheet typical) [1/degC]
P.ctrl.hall_a = 1;  P.ctrl.hall_b = 0;  P.ctrl.T_cal = P.T0;
% Initial write-voltage table = the device LUT. Refined online after every write.
P.ctrl.cal_f = P.lut.f;   % [Hz]  (must be increasing)
P.ctrl.cal_V = P.lut.V;   % [V]
end
