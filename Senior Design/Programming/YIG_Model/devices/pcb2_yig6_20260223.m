function dev = pcb2_yig6_20260223()
%PCB2_YIG6_20260223  Vendor calibration LUT: PCB2, YIG6, new passive, 2026-02-23.
%   Procedure: reset with a VOL 50 pulse on POL 7A, then one write pulse
%   of the voltage below on POL 7B. Measured on S12.
%   Note the polarity codes are swapped relative to PCB1.

dev.name      = 'PCB2-YIG6-20260223 (new passive)';
dev.sparam    = 'S12';
dev.pol_reset = '7A';
dev.pol_write = '7B';
dev.V_reset   = 50;                   % [V]
dev.f         = [4.6 7.2 10.0 12.8]*1e9;  % [Hz]
dev.V         = [20  25   30   35];       % [V]
end
