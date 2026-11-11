function mag = yig_magnet_step(I_coil, P)
%YIG_MAGNET_STEP  AlNiCo latching magnet: Preisach hysteresis + gap demag.
%   One plant step.  Codegen-compatible.
%
%   I_coil : main-coil current [A]
%   mag    : [m; H_eff; H_coil]
%            m      normalised magnetisation (-1..1), sets the gap field
%            H_eff  field the magnet actually sees  = H_coil - D*m  [A/m]
%            H_coil applied coil field = kH*I_coil [A/m]
%
%   Model: classical scalar Preisach with an Everett table (P.mag.E) and a
%   memory stack of alternating dominant extrema (wipe-out property).
%   Because the magnet sits in an open magnetic circuit, it sees its own
%   demagnetising field.  H_eff is solved implicitly every step so the
%   sheared loop / recoil behaviour is exact, independent of step size.

persistent seq n x_prev H_prev m_now
if isempty(seq)
    [seq, n] = pr_init(P);
    x_prev = 0;  H_prev = 0;
    % initial self-consistent state at H = 0
    [x_prev, seq, n, m_now] = solve_heff(0, -P.mag.D, P.mag.D, seq, n, P);
end

H = P.mag.kH * I_coil;
if H ~= H_prev
    a = x_prev;  b = x_prev + (H - H_prev);   % |dx| <= |dH|, same sign
    [x_prev, seq, n, m_now] = solve_heff(H, a, b, seq, n, P);
    H_prev = H;
end
mag = [m_now; x_prev; H];
end

%% ======================================================================
function [x, seq, n, m] = solve_heff(H, a, b, seq0, n0, P)
% find x with  g(x) = x + D*m(x) - H = 0   (g is monotone increasing)
D = P.mag.D;
lo = min(a, b);  hi = max(a, b);
glo = gfun(lo, H, seq0, n0, P);
ghi = gfun(hi, H, seq0, n0, P);
if glo > 0 || ghi < 0          % bracket failed -> use the safe one
    lo = H - D;  hi = H + D;
    glo = gfun(lo, H, seq0, n0, P);
    ghi = gfun(hi, H, seq0, n0, P);
end
x = 0.5*(lo + hi);
side = 0;
for it = 1:60
    if hi - lo < 1e-3, break; end
    if ghi ~= glo
        x = (lo*ghi - hi*glo) / (ghi - glo);   % regula falsi
    else
        x = 0.5*(lo + hi);
    end
    if x <= lo || x >= hi, x = 0.5*(lo + hi); end
    gx = gfun(x, H, seq0, n0, P);
    if abs(gx) < 1e-3, break; end
    if gx > 0
        hi = x;  ghi = gx;
        if side == -1, glo = glo/2; end   % Illinois modification
        side = -1;
    else
        lo = x;  glo = gx;
        if side == 1, ghi = ghi/2; end
        side = 1;
    end
end
[seq, n] = pr_update(seq0, n0, x, P);
m = pr_out(seq, n, P);
end

function g = gfun(x, H, seq, n, P)
[s2, n2] = pr_update(seq, n, x, P);
g = x + P.mag.D*pr_out(s2, n2, P) - H;
end

%% ======================================================================
function [seq, n] = pr_init(P)
NMAX = 64;                       % memory depth (fixed size for codegen)
seq = zeros(NMAX, 1);
Hm = P.mag.Hm;
if P.mag.init > 0
    n = 1;  seq(1) = Hm;                 % positive saturation
elseif P.mag.init < 0
    n = 0;                               % negative saturation
else
    % AC-demagnetised: decaying alternating history
    n = min(NMAX - 4, 60);
    n = n - mod(n, 2);
    r = 0.9;
    for k = 1:n
        amp = Hm * r^(k-1);
        if mod(k, 2) == 1
            seq(k) = amp;
        else
            seq(k) = -amp;
        end
    end
end
end

function [seq, n] = pr_update(seq, n, u, P)
% apply input u to the Preisach memory (stack of alternating extrema)
Hm = P.mag.Hm;  ep = P.mag.eps_rev;  NMAX = numel(seq);
u = min(max(u, -Hm), Hm);
if n == 0                                    % negative saturation
    if u > -Hm + ep
        n = 1;  seq(1) = u;
    end
elseif mod(n, 2) == 1                        % last move was increasing
    if u >= seq(n)
        seq(n) = u;
        while n >= 3 && seq(n) >= seq(n-2)   % wipe out smaller maxima
            seq(n-2) = seq(n);  n = n - 2;
        end
    elseif u < seq(n) - ep && n < NMAX       % reversal -> new minimum
        n = n + 1;  seq(n) = u;
        [seq, n] = wipe_min(seq, n, Hm);
    end
else                                         % last move was decreasing
    if u <= seq(n)
        seq(n) = u;
        [seq, n] = wipe_min(seq, n, Hm);
    elseif u > seq(n) + ep && n < NMAX       % reversal -> new maximum
        n = n + 1;  seq(n) = u;
        while n >= 3 && seq(n) >= seq(n-2)
            seq(n-2) = seq(n);  n = n - 2;
        end
    end
end
end

function [seq, n] = wipe_min(seq, n, Hm)
while n >= 4 && seq(n) <= seq(n-2)
    seq(n-2) = seq(n);  n = n - 2;
end
if n == 2 && seq(2) <= -Hm
    n = 0;
end
end

function m = pr_out(seq, n, P)
% m = -1 + 2*sum_k [E(M_k, m_{k-1}) - E(M_k, m_k)]
m = -1;
prevmin = -P.mag.Hm;
for k = 1:2:n
    M = seq(k);
    m = m + 2*everett(M, prevmin, P);
    if k + 1 <= n
        mn = seq(k+1);
        m = m - 2*everett(M, mn, P);
        prevmin = mn;
    end
end
end

function e = everett(a, b, P)
% bilinear interpolation of the normalised Everett table E(alpha,beta)
Hm = P.mag.Hm;  dH = P.mag.dH;  Ng = P.mag.Ng;
a = min(max(a, -Hm), Hm);
b = min(max(b, -Hm), Hm);
if a <= b
    e = 0;
    return;
end
fa = (a + Hm)/dH + 1;   fb = (b + Hm)/dH + 1;
ia = min(floor(fa), Ng - 1);  ib = min(floor(fb), Ng - 1);
ta = fa - ia;  tb = fb - ib;
E = P.mag.E;
e = (1-ta)*(1-tb)*E(ia, ib)   + ta*(1-tb)*E(ia+1, ib) + ...
    (1-ta)*tb    *E(ia, ib+1) + ta*tb    *E(ia+1, ib+1);
end
