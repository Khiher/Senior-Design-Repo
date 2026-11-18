function dev = pcb2_old_20250611()
%PCB2_OLD_20250611  Vendor calibration LUT: PCB2, old passive, 2025-06-11.
%   Procedure: reset with a VOL 75 pulse on POL 7A, then one write pulse
%   of the voltage below on POL 7B. Measured on S21.

dev.name      = 'PCB2-20250611 (old passive)';
dev.sparam    = 'S21';
dev.pol_reset = '7A';
dev.pol_write = '7B';
dev.V_reset   = 75;                   % [V]
dev.f         = [4.6 7.0 10.0 13.0]*1e9;  % [Hz]
dev.V         = [20  27   36   50];       % [V]
end
