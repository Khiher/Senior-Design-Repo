function y = yig_sensors_step(drv, res, env, P)
%YIG_SENSORS_STEP  What the controller can actually measure.
%   Adds noise, drift and ADC quantisation to the plant states.
%   Codegen-compatible.
%
%   y : measurement vector (8x1) handed to the controller
%       y(1) I_meas     main-coil current (shunt)          [A]
%       y(2) Vc_meas    storage-cap voltage                [V]
%       y(3) B_hall     Hall-sensor reading, ADC output    [T]
%                       (the controller converts it to frequency with its
%                        bench calibration, see yig_calibrate_hall)
%       y(4) P_det      detector power at probe frequency  [dBm]
%       y(5) T_meas     magnet temperature                 [degC]
%       y(6) busy       1 while a pulse is in progress
%       y(7) ready      1 when idle and cap is at V0_set
%       y(8) Itrim_meas trim-coil current                  [A]

persistent hall_lp
S = P.sens;
T = env(1);
B_hall = yig_hall_ideal(res(2), T, P);      % deterministic part
if isempty(hall_lp), hall_lp = B_hall; end
hall_lp = hall_lp + P.sim.Ts/P.hall.tau*(B_hall - hall_lp);   % sensor bandwidth
B_read = hall_lp + P.hall.noise_sd*randn();
lsb = 2*P.hall.FS/2^P.hall.bits;
B_read = lsb*round(min(max(B_read, -P.hall.FS), P.hall.FS)/lsb);

I_meas = S.I_lsb*round((drv(1) + S.I_sd*randn())/S.I_lsb);
V_meas = S.V_lsb*round((drv(2) + S.V_sd*randn())/S.V_lsb);
P_det  = max(res(3) + S.P_sd*randn(), P.yig.floor_dBm);
T_meas = T + S.T_sd*randn();

y = [I_meas; V_meas; B_read; P_det; T_meas; drv(3); drv(4); drv(5)];
end
