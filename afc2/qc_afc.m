function [u, info] = qc_afc(C, t, x)
%QC_AFC Approximation-free control law, Na et al. eqs. (9)-(20).
%   mu_i(t)  = (mu_i0 - mu_inf)exp(-alpha_i t) + mu_inf          (9)
%   zeta_i   = e_i / mu_i                                          (21)
%   eps_i    = 0.5 ln((delta + zeta_i)/(dbar - zeta_i))            (13)
%   e1 = x1,        u1 = -k1 eps1                                  (14)-(15)
%   e2 = x2 - u1,   u2 = -k2 eps2                                  (16)-(17)
%   e3 = x5 - u2,   u  = -k3 eps3                                  (18)-(20)
%
%   info.zeta holds the *unclipped* normalized errors, so |zeta_i| >= 1
%   flags a real prescribed-performance violation; the log itself is
%   evaluated on a guarded value so the code never produces Inf.
mu = (C.mu0 - C.muinf) .* exp(-C.alpha * t) + C.muinf;
if isfield(C, 'rho')
    mu = mu * (1 + C.rho);   % envelope relaxation (afc_aw extension only)
end
e = zeros(3, 1);
zeta = zeros(3, 1);
ep = zeros(3, 1);
v = zeros(3, 1);
clipped = false(3, 1);
src = [x(1); x(2); x(5)];
prev = 0;
for i = 1:3
    e(i) = src(i) - prev;
    zeta(i) = e(i) / mu(i);
    zg = min(max(zeta(i), -C.delta + C.guard), C.dbar - C.guard);
    clipped(i) = (zg ~= zeta(i));
    ep(i) = 0.5 * log((C.delta + zg) / (C.dbar - zg));
    v(i) = -C.k(i) * ep(i);
    prev = v(i);
end
u = v(3);
info.mu = mu;
info.e = e;
info.zeta = zeta;
info.eps = ep;
info.v = v;
info.clipped = clipped;
end
