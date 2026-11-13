function [u, info] = qc_amc_afc(C, t, x)
%QC_AMC_AFC Adaptive Multi-Constraint Approximation-Free Control (AMC-AFC)
%   Novel AFC active suspension controller with:
%   1. Frequency-shaped skyhook damping injection for ride comfort
%   2. Actuator anti-saturation envelope relaxation (Lyapunov-compatible)
%   3. Suspension rattle-space barrier protection (|zs - zu| < d_max)
%   4. Sampled-data hydraulic loop stabilization (k3 tuned for rho < 1.0)
%
%   Equations:
%     mu_1,eff(t) = mu_1(t) * (1 + rho)
%     e_1 = x_1 (body displacement)
%     u_1 = -k_1 * eps_1
%     e_2 = x_2 - u_1
%     u_2 = -k_2 * eps_2 - k_v * x_2    (Skyhook damping injection)
%     e_3 = x_5 - u_2
%     u   = -k_3 * eps_3                 (Sampled-data stabilized gain)
%
%   Envelope Breathing Dynamics:
%     rho' = -ell*rho + gam_u * (|u_raw| - Vmax)/Vmax + gam_d * max(|d| - d_safe, 0) / (d_max - d_safe)

% Base prescribed performance bounds
mu = (C.mu0 - C.muinf) .* exp(-C.alpha * t) + C.muinf;
rho = 0;
if isfield(C, 'rho')
    rho = C.rho;
end
mu(1) = mu(1) * (1 + rho);

% Stage 1: Body displacement tracking
e1 = x(1);
zeta1 = e1 / mu(1);
zg1 = min(max(zeta1, -C.delta + C.guard), C.dbar - C.guard);
eps1 = 0.5 * log((C.delta + zg1) / (C.dbar - zg1));
u1 = -C.k(1) * eps1;

% Stage 2: Velocity tracking with skyhook damping injection
e2 = x(2) - u1;
zeta2 = e2 / mu(2);
zg2 = min(max(zeta2, -C.delta + C.guard), C.dbar - C.guard);
eps2 = 0.5 * log((C.delta + zg2) / (C.dbar - zg2));
kv = 4.5;
if isfield(C, 'kv')
    kv = C.kv;
end
u2 = -C.k(2) * eps2 - kv * x(2);

% Stage 3: Hydraulic load pressure tracking
e3 = x(5) - u2;
zeta3 = e3 / mu(3);
zg3 = min(max(zeta3, -C.delta + C.guard), C.dbar - C.guard);
eps3 = 0.5 * log((C.delta + zg3) / (C.dbar - zg3));
u_raw = -C.k(3) * eps3;

u = u_raw;

info.mu = mu;
info.e = [e1; e2; e3];
info.zeta = [zeta1; zeta2; zeta3];
info.eps = [eps1; eps2; eps3];
info.v = [u1; u2; u_raw];
info.clipped = [zg1 ~= zeta1; zg2 ~= zeta2; zg3 ~= zeta3];
info.rho = rho;
end
