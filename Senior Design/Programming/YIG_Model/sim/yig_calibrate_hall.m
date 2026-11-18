function P = yig_calibrate_hall(P, rep, verbose)
%YIG_CALIBRATE_HALL  Simulated bench calibration of the Hall sensor.
%
%   P = yig_calibrate_hall(P, rep)   rep from yig_fit_device
%
%   On the bench you would write the magnet to several frequencies at a
%   fixed temperature, read the true centre frequency on the VNA and the
%   Hall output, and fit  f = hall_a*B_hall + hall_b.  This replays that at
%   P.T0 over the device's tuning range (reset state up to the strongest
%   write), averaging away the sensor noise. Because f and the Hall output
%   are both linear in the gap field, the fit removes the sensor's coupling
%   factor, gain error and offset exactly at the calibration temperature.
%
%   What calibration can NOT remove is temperature drift of the sensor.
%   The firmware divides by (1 + hall_tc_comp*(T - T_cal)) using the
%   datasheet's typical gain tempco; the difference from the true tempco
%   (P.hall.tc_gain) and the offset drift (P.hall.tc_offset) remain.

if nargin < 3, verbose = true; end
f_cal = linspace(rep.f_reset, rep.f_wmax, 8);
B_gap = (f_cal - P.yig.f_anis)/P.yig.gamma;
B_rd  = yig_hall_ideal(B_gap, P.T0, P);
c = polyfit(B_rd, f_cal, 1);
P.ctrl.hall_a = c(1);           % [Hz/T]
P.ctrl.hall_b = c(2);           % [Hz]
P.ctrl.T_cal  = P.T0;           % [degC]

if verbose
    g = P.hall.tc_gain - P.ctrl.hall_tc_comp;
    fprintf(['Hall calibrated at %g degC. Uncompensated drift: gain %+.0f ppm/degC ' ...
             '(about %+.1f MHz/degC at 13 GHz), offset %.2f G/degC (about %.1f MHz/degC).\n'], ...
        P.T0, g*1e6, g*(13e9 - P.yig.f_anis)/1e6, P.hall.tc_offset*1e4, ...
        P.ctrl.hall_a*P.hall.tc_offset/1e6);
end
end
