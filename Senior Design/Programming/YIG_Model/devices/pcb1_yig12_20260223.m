function dev = pcb1_yig12_20260223()
%PCB1_YIG12_20260223  Vendor calibration LUT: PCB1, YIG12, new passive, 2026-02-23.
%   Procedure: reset with a VOL 50 pulse on POL 7B, then one write pulse
%   of the voltage below on POL 7A. Measured on S12.
%   See "Device LUTs" in README.md for the struct format.

dev.name      = 'PCB1-YIG12-20260223 (new passive)';
dev.sparam    = 'S12';
dev.pol_reset = '7B';                 % H-bridge code that drives the field DOWN
dev.pol_write = '7A';                 % H-bridge code that drives the field UP
dev.V_reset   = 50;                   % [V]
dev.f         = [4.5 7.0 10.0 12.8]*1e9;  % [Hz]
dev.V         = [20  25   30   35];       % [V]
end
