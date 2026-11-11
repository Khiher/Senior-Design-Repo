function P = yig_build_everett(P)
%YIG_BUILD_EVERETT  (Re)build the Preisach Everett table from P.mag.
%   Call after changing hc_mean, hc_sd or hu_sd.
%
%   E(alpha,beta) is normalised so that E(Hm,-Hm) = 1. Bilinear
%   interpolation of this table gives a smooth, continuous hysteresis
%   model with correct minor loops and wipe-out.

Hm   = P.mag.hc_mean + 4*P.mag.hc_sd + 4*P.mag.hu_sd;   % support limit
Ng   = 241;
h    = linspace(-Hm, Hm, Ng);
[A, B] = ndgrid(h, h);                 % A = alpha (rows), B = beta (cols)
hc   = (A - B)/2;   hu = (A + B)/2;
mu   = exp(-0.5*((hc - P.mag.hc_mean)/P.mag.hc_sd).^2) .* ...
       exp(-0.5*(hu/P.mag.hu_sd).^2);
mu(A < B) = 0;
% E(i,j) = sum_{a<=i} sum_{b>=j} mu(a,b)
E = cumsum(mu, 1);
E = fliplr(cumsum(fliplr(E), 2));
E = E / E(end, 1);
P.mag.Hm   = Hm;
P.mag.Ng   = Ng;
P.mag.dH   = h(2) - h(1);
P.mag.E    = E;
end
