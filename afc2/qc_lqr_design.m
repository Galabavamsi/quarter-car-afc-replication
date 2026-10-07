function C = qc_lqr_design(P, w)
%QC_LQR_DESIGN Discrete-time LQR comparator designed on the linearised plant.
%   C = qc_lqr_design(P, w) with weights (all optional)
%       w.qz   weight on body displacement z_s^2        (default 1e4)
%       w.qa   weight on body acceleration  zdd_s^2     (default 1)
%       w.qt   weight on suspension travel (z_s-z_u)^2  (default 0)
%       w.r    weight on valve voltage u^2              (default 1)
%   The plant is discretised with a zero-order hold at the 50 Hz controller
%   rate (matrix exponential), the cost is the sampled version of
%       J = sum qz z_s^2 + qa a_s^2 + qt travel^2 + r u^2
%   and the discrete Riccati equation is solved by fixed-point iteration
%   (no Control System Toolbox needed). Full-state feedback u = -K x, so the
%   comparison is with the same information as AFC (which uses x1, x2, x5).
if nargin < 2
    w = struct();
end
qz = getw(w, 'qz', 1e4);
qa = getw(w, 'qa', 1);
qt = getw(w, 'qt', 0);
r = getw(w, 'r', 1);
Lin = qc_lin_cont(P);
n = Lin.n;
Ts = P.sim.Ts;
M = expm([Lin.A, Lin.B; zeros(1, n + 1)] * Ts);
Ad = M(1:n, 1:n);
Bd = M(1:n, n + 1);
% cost in terms of x and u: a_s = Ca x + Da u
Q = qz * (Lin.Cz' * Lin.Cz) + qa * (Lin.Ca' * Lin.Ca) + qt * (Lin.Ctravel' * Lin.Ctravel);
N = qa * Lin.Ca' * Lin.Da;
R = r + qa * Lin.Da ^ 2;
X = Q;
for it = 1:20000
    G = R + Bd' * X * Bd;
    K = G \ (Bd' * X * Ad + N');
    Xn = Ad' * X * Ad - (Ad' * X * Bd + N) * K + Q;
    if norm(Xn - X, 'fro') <= 1e-10 * max(1, norm(X, 'fro'))
        X = Xn;
        break;
    end
    X = Xn;
end
K = (R + Bd' * X * Bd) \ (Bd' * X * Ad + N');
C.type = 'lqr';
C.variant = 'A';
C.K = K;
C.n = n;
C.weights = struct('qz', qz, 'qa', qa, 'qt', qt, 'r', r);
C.rho = max(abs(eig(Ad - Bd * K)));
C.label = 'LQR';
end

function v = getw(w, name, d)
if isfield(w, name)
    v = w.(name);
else
    v = d;
end
end
