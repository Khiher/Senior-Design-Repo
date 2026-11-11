function [P, rep] = yig_fit_device(P, verbose)
%YIG_FIT_DEVICE  Fit the magnet model so it reproduces the device LUT.
%
%   [P, rep] = yig_fit_device(P)        P from yig_params(dev)
%   [P, rep] = yig_fit_device(P, false) quiet
%
%   For every LUT point (V, f) the vendor procedure is replayed through the
%   real plant code: a natural reset pulse at -V_reset, then a natural
%   write pulse at +V. These parameters are fitted so the resulting centre
%   frequencies match the LUT:
%       P.gap.B_pass   passive bias field         [T]
%       P.gap.Bmax     AlNiCo field at m = 1      [T]
%       P.mag.hc_mean  mean coercivity            [A/m]
%       P.mag.hc_sd    coercivity spread          [A/m]
%       P.mag.hu_sd    interaction-field spread   [A/m]  (weakly held near its start value)
%       P.mag.D        demagnetising field        [A/m]  (weakly held near its start value)
%   with two physical constraints:
%       - the reset pulse must fully saturate the magnet (so every write
%         starts from the same state, which is what makes a LUT valid)
%       - the field after reset stays positive (f(V) stays monotone)
%
%   rep : struct with the LUT, the fitted frequencies, the RMS error, the
%         frequency right after a reset and at the strongest allowed write.

if nargin < 2, verbose = true; end

% Peak coil currents of natural pulses (no I_lim) for every voltage needed
Vw = P.lut.V(:);
Ir = pulse_peak(P, P.lut.V_reset);
Iw = arrayfun(@(v) pulse_peak(P, v), Vw);
Imax_w = pulse_peak(P, P.ctrl.V_write_max);

x0s = [P.gap.B_pass, P.gap.Bmax, P.mag.hc_mean, P.mag.hc_sd, P.mag.hu_sd, P.mag.D];
opts = optimset('Display', 'off', 'TolX', 1e-6, 'TolFun', 1e-6, ...
                'MaxFunEvals', 4000, 'MaxIter', 4000);
best = inf;  xb = log(x0s);
% a few starts on coercivity, since it sets where along V the curve rises
for hc0 = [0.8 1.0 1.25] * x0s(3)
    x0 = log([x0s(1:2), hc0, x0s(4:end)]);
    [x, c] = fminsearch(@(x) cost(x, P, Ir, Iw, x0s), x0, opts);
    if c < best, best = c;  xb = x; end
end
P = apply(xb, P);

[~, f_fit] = cost(xb, P, Ir, Iw, x0s);
rep.f_lut   = P.lut.f(:);
rep.V_lut   = Vw;
rep.f_fit   = f_fit;
rep.rms_MHz = sqrt(mean((f_fit - rep.f_lut).^2))/1e6;
rep.f_reset = freq_after(P, Ir, 0);          % reset only (no write)
rep.f_wmax  = freq_after(P, Ir, Imax_w);     % strongest allowed write
rep.I_reset = Ir;
rep.I_write = Iw;

% ---- reverse-pulse calibration (for strategy 2) -------------------------
% Bench equivalent: write at each voltage on a 1 V grid, then fire
% increasing reverse pulses and record the voltage that lowers the
% frequency by each amount in rev_D.
D_grid = [5 10 20 40 80]*1e6;
Vg = (floor(min(Vw)) - 1):1:min(ceil(max(Vw)) + 3, P.ctrl.V_write_max);
ipv = Iw(end)/Vw(end);                       % peak amps per volt (natural pulse, linear)
rev_V = zeros(numel(Vg), numel(D_grid));
for k = 1:numel(Vg)
    fw = freq_after(P, Ir, ipv*Vg(k));
    for j = 1:numel(D_grid)
        lo = 0;  hi = P.ctrl.V_reset;            % drop(lo) = 0, drop(V_reset) = everything
        for it = 1:30                            % bisection (drop is monotone in the reverse V)
            mid = 0.5*(lo + hi);
            if fw - freq_after(P, Ir, ipv*Vg(k), ipv*mid) < D_grid(j), lo = mid; else, hi = mid; end
        end
        rev_V(k, j) = 0.5*(lo + hi);
    end
end
P.ctrl.rev_Vw = Vg;   P.ctrl.rev_D = D_grid;   P.ctrl.rev_V = rev_V;
rep.rev_V = rev_V;

if verbose
    fprintf(['Fitted magnet to LUT:  B_pass = %.4f T, Bmax = %.4f T, hc_mean = %.1f kA/m, ' ...
             'hc_sd = %.1f kA/m, hu_sd = %.1f kA/m, D = %.1f kA/m\n'], ...
        P.gap.B_pass, P.gap.Bmax, P.mag.hc_mean/1e3, P.mag.hc_sd/1e3, P.mag.hu_sd/1e3, P.mag.D/1e3);
    fprintf('   V [V]   I_pk [A]   LUT [GHz]   model [GHz]   error [MHz]\n');
    for k = 1:numel(Vw)
        fprintf('   %5.1f   %7.2f    %8.3f    %9.3f    %+9.1f\n', Vw(k), Iw(k), ...
            rep.f_lut(k)/1e9, f_fit(k)/1e9, (f_fit(k) - rep.f_lut(k))/1e6);
    end
    fprintf('   RMS error %.1f MHz.  After reset only: %.2f GHz.  Write at %g V: %.2f GHz.\n', ...
        rep.rms_MHz, rep.f_reset/1e9, P.ctrl.V_write_max, rep.f_wmax/1e9);
    fprintf('   Reverse pulse [V] that lowers f by %s MHz, after a write at:\n', mat2str(D_grid/1e6));
    for k = 1:numel(Vg)
        if any(abs(Vg(k) - Vw) < 0.5)
            fprintf('     +%4.1f V: %s\n', Vg(k), mat2str(round(rev_V(k,:), 2)));
        end
    end
end
end

%% ======================================================================
function P = apply(x, P)
v = exp(x);
P.gap.B_pass  = v(1);
P.gap.Bmax    = v(2);
P.mag.hc_mean = v(3);
P.mag.hc_sd   = v(4);
P.mag.hu_sd   = v(5);
P.mag.D       = v(6);
P = yig_build_everett(P);
end

function [c, f] = cost(x, P, Ir, Iw, x0s)
P = apply(x, P);
f = zeros(numel(Iw), 1);
for k = 1:numel(Iw)
    f(k) = freq_after(P, Ir, Iw(k));
end
c = sum(((f - P.lut.f(:))/1e6).^2);                      % [MHz^2]
% reset must saturate: effective reverse field beyond (almost) all hysterons
H_need = P.mag.hc_mean + 3*P.mag.hc_sd + 3*P.mag.hu_sd + P.mag.D;
c = c + 1e4*max(0, (H_need - P.mag.kH*Ir)/1e3)^2;
% field after reset must stay positive
c = c + 1e4*max(0, (P.gap.Bmax - P.gap.B_pass + 0.005)/0.001)^2;
% weak pull of hu_sd and D toward their start values (they are poorly
% determined by 4 LUT points): a factor-of-2 change costs ~1 MHz^2
c = c + 2*sum(log(exp(x(5:6))./x0s(5:6)).^2);
end

function f = freq_after(P, Ir, Iw, Irev)
% replay: reset pulse (-Ir) -> 0 -> write pulse (+Iw) -> 0
%         [-> reverse pulse (-Irev) -> 0], read the field
yig_magnet_step(0, P, true);           % fresh magnet (P.mag.init state)
yig_magnet_step(-Ir, P);
mag = yig_magnet_step(0, P);
if Iw > 0
    yig_magnet_step(Iw, P);
    mag = yig_magnet_step(0, P);
end
if nargin > 3 && Irev > 0
    yig_magnet_step(-Irev, P);
    mag = yig_magnet_step(0, P);
end
B = P.gap.B_pass + P.gap.Bmax*mag(1);
f = P.yig.gamma*abs(B) + P.yig.f_anis;
end

function Ipk = pulse_peak(P, V)
% natural pulse from a cap charged to V: run the real driver code
P.drv.Vset_err_sd = 0;  P.drv.Vc0 = V;
clear yig_driver_step
u = [V; 0; 1; 0; 0; 0];
yig_driver_step(u, P);                 % idle, cap at V
u(2) = 1;
Ipk = 0;
for k = 1:2000
    d = yig_driver_step(u, P);
    Ipk = max(Ipk, abs(d(1)));
    if k > 2 && d(3) < 0.5, break; end
end
clear yig_driver_step
end
