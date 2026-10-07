function L = qc_linearize(P, C, tEval)
%QC_LINEARIZE Local stability of the sampled-data closed loop at the origin.
%   L = qc_linearize(P, C) numerically differentiates the one-sample map
%       x(k+1) = Phi(x(k))   (50 Hz ZOH controller + RK4 plant, no road,
%                             no voltage rail)
%   at x = 0 with the controller evaluated at tEval (default 100 s, i.e.
%   the PPFs have settled at mu_inf).
%   L.Ad        Jacobian (6x6, or 5x5 when the spool state is unused)
%   L.eig_d     discrete eigenvalues,  L.rho = spectral radius (stable < 1)
%   L.eig_c     continuous-equivalent poles log(eig_d)/Ts
%   L.K         equivalent linear feedback u = K*x at the origin (AFC)
if nargin < 3
    tEval = 100;
end
useSpool = strcmp(P.act.model, 'valve') && P.act.tau > 0;
nx = 5 + useSpool;
R = qc_road('sine', 0, 1);
Ts = P.sim.Ts;
sub = max(1, round(Ts / P.sim.h));
h = Ts / sub;
del = 1e-7 * [1 1 1 1 1 1e-3]';   % spool state is ~1e-4 m full scale
A = zeros(nx);
K = zeros(1, nx);
for i = 1:nx
    dx = zeros(6, 1);
    dx(i) = del(i);
    [xp, up] = step_map(P, C, R, dx, tEval, h, sub);
    [xm, um] = step_map(P, C, R, -dx, tEval, h, sub);
    A(:, i) = (xp(1:nx) - xm(1:nx)) / (2 * del(i));
    K(i) = (up - um) / (2 * del(i));
end
L.Ad = A;
L.eig_d = eig(A);
L.rho = max(abs(L.eig_d));
L.eig_c = log(L.eig_d) / Ts;
L.K = K;
L.stable = L.rho < 1;
end

function [xk, u] = step_map(P, C, R, x0, tEval, h, sub)
if strcmp(C.type, 'pid')
    C.I = 0;
end
[u, ~, ~] = qc_controller(C, tEval, x0, P, 0, 0);
xk = x0;
passive = strcmp(C.type, 'passive');
for j = 1:sub
    k1 = qc_plant_rhs(P, xk, u, 0, 0, passive);
    k2 = qc_plant_rhs(P, xk + h / 2 * k1, u, 0, 0, passive);
    k3 = qc_plant_rhs(P, xk + h / 2 * k2, u, 0, 0, passive);
    k4 = qc_plant_rhs(P, xk + h * k3, u, 0, 0, passive);
    xk = xk + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
end
end
