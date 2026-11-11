function y = yig_sensors_step(drv, res, env, P)
%YIG_SENSORS_STEP  What the controller can actually measure.
%   Adds noise and ADC quantisation to the plant states.  Codegen-compatible.
%
%   y : measurement vector (8x1) handed to the controller
%       y(1) I_meas     main-coil current (shunt)          [A]
%       y(2) Vc_meas    storage-cap voltage                [V]
%       y(3) f_meas     filter centre-frequency estimate   [Hz]
%       y(4) P_det      detector power at probe frequency  [dBm]
%       y(5) T_meas     magnet temperature                 [degC]
%       y(6) busy       1 while a pulse is in progress
%       y(7) ready      1 when idle and cap is at V0_set
%       y(8) Itrim_meas trim-coil current                  [A]

S = P.sens;
I_meas = S.I_lsb*round((drv(1) + S.I_sd*randn())/S.I_lsb);
V_meas = S.V_lsb*round((drv(2) + S.V_sd*randn())/S.V_lsb);
f_meas = res(1) + S.f_sd*randn();
P_det  = max(res(3) + S.P_sd*randn(), P.yig.floor_dBm);
T_meas = env(1) + S.T_sd*randn();

y = [I_meas; V_meas; f_meas; P_det; T_meas; drv(3); drv(4); drv(5)];
end
