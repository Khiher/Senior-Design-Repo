function B_out = yig_hall_ideal(B_gap, T, P)
%YIG_HALL_IDEAL  Noise-free Hall-sensor reading for a gap field B_gap at T.
%   Codegen-compatible. Used by yig_sensors_step (plus noise, bandwidth and
%   ADC) and by yig_calibrate_hall (the bench calibration).
%
%   The sensor sits near, not at, the YIG sphere, so it sees a fraction
%   k of the gap field. Its gain and offset are wrong by fixed amounts
%   (removed by calibration) and drift with temperature (only partly
%   removed by the firmware's temperature compensation).
H  = P.hall;
dT = T - P.T0;
B_out = (1 + H.gain_err)*(1 + H.tc_gain*dT)*H.k*B_gap + H.offset + H.tc_offset*dT;
end
