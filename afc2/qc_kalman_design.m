function KF = qc_kalman_design(P, sig)
%QC_KALMAN_DESIGN Steady-state Kalman filter with the paper's two sensors.
%   The paper measures only body displacement z_s and body acceleration
%   a_s (50 Hz) and estimates the cylinder force with an extended state
%   observer. This is the closest reproducible equivalent: a discrete
%   steady-state Kalman filter on the linearised plant, augmented with the
%   road displacement as a random-walk disturbance state, using exactly
%   the same two measurements. It returns estimates of all states, of
%   which AFC uses z_s, zdot_s and x5.
%
%   sig.z    displacement sensor std (m)          default 0.5e-3
%   sig.a    accelerometer std (m/s^2)            default 0.05
%   sig.road road random-walk std per sample (m)  default 1e-2
%   sig.proc relative process noise on states     default 5e-2
%   (defaults chosen by a small tuning sweep, see run_phase2 section B)
if nargin < 2
    sig = struct();
end
sz = getf(sig, 'z', 0.5e-3);
sa = getf(sig, 'a', 0.05);
sr = getf(sig, 'road', 1e-2);
sp = getf(sig, 'proc', 5e-2);
Lin = qc_lin_cont(P);
n = Lin.n;
Ts = P.sim.Ts;
% augmented continuous model, state [x; zr]
Ac = [Lin.A, Lin.E; zeros(1, n + 1)];
Bc = [Lin.B; 0];
M = expm([Ac, Bc; zeros(1, n + 2)] * Ts);
Ad = M(1:n + 1, 1:n + 1);
Bd = M(1:n + 1, n + 2);
C = [Lin.Cz, 0; Lin.Ca, Lin.Ea];
D = [0; Lin.Da];
scale = [0.05, 0.3, 0.05, 0.5, 2, 1e-4];   % typical state magnitudes
Q = diag([(sp * scale(1:n)) .^ 2, sr ^ 2]);
R = diag([sz ^ 2, sa ^ 2]);
X = eye(n + 1) * 1e-4;
for it = 1:50000
    S = C * X * C' + R;
    Xn = Ad * X * Ad' - Ad * X * C' / S * C * X * Ad' + Q;
    if norm(Xn - X, 'fro') <= 1e-12 * max(1, norm(X, 'fro'))
        X = Xn;
        break;
    end
    X = Xn;
end
KF.Mk = X * C' / (C * X * C' + R);     % measurement-update gain
KF.Ad = Ad;
KF.Bd = Bd;
KF.C = C;
KF.D = D;
KF.n = n;
KF.sig = struct('z', sz, 'a', sa, 'road', sr, 'proc', sp);
KF.rho_est = max(abs(eig((eye(n + 1) - KF.Mk * C) * Ad)));
end

function v = getf(s, f, d)
if isfield(s, f)
    v = s.(f);
else
    v = d;
end
end
