function run = qc_halfcar(ctrlType, roadKind, opts)
%QC_HALFCAR Half-car (heave + pitch) extension, paper Remark 3.
%   run = qc_halfcar(ctrlType, roadKind) with
%       ctrlType: 'passive' | 'afc' | 'afc_aw' | 'skyhook'
%       roadKind: 'bump' (50 mm x 2.5 m at 20 km/h, rear wheel hit
%                 (a+b)/v later) | 'case10' (paper sine, same delay)
%   Two calibrated servo-valve corners (qc_params('alleyne-cal')) carry a
%   rigid body of mass 2*290 kg and pitch inertia I. Each corner runs its
%   own decentralised AFC (Case A gains) on its local body-corner states,
%   exactly the quarter-car law, so the extension needs no new model
%   knowledge. States (12):
%     [zc zc' th th' zuf zuf' zur zur' x5f xvf x5r xvr]
%   Corner displacement: z_sf = zc - a*th, z_sr = zc + b*th.
if nargin < 3
    opts = struct();
end
Pq = qc_params('alleyne-cal');
H.a = 1.2; H.b = 1.4; H.M = 2 * Pq.mech.ms; H.I = 1100; H.v = 20 / 3.6;
if isfield(opts, 'I'), H.I = opts.I; end
m = Pq.mech; a = Pq.act;
Ts = Pq.sim.Ts; h = Pq.sim.h; sub = round(Ts / h);
T = 6;
if strcmp(roadKind, 'case10'), T = 20; end
if isfield(opts, 'T'), T = opts.T; end
N = round(T / Ts);
delay = (H.a + H.b) / H.v;
switch roadKind
    case 'bump'
        Rb = qc_road('bump', 0.05, 2.5, H.v, 1.0);
    case 'case10'
        Rb = qc_road('case', 10);
    otherwise
        error('qc_halfcar:road', 'roadKind must be bump or case10');
end
road = @(t) road_pair(Rb, t, delay);
Cs = {qc_ctrl('afc', 'A'), qc_ctrl('afc', 'A')};
if strcmp(ctrlType, 'afc_aw')
    Cs = {qc_ctrl('afc_aw', 'A'), qc_ctrl('afc_aw', 'A')};
end
passive = strcmp(ctrlType, 'passive');
x = zeros(12, 1);
t = (0:N)' * Ts;
X = nan(12, N + 1); X(:, 1) = x;
U = zeros(2, N + 1); Uraw = U; ACC = zeros(2, N + 1); ZR = zeros(2, N + 1);
for k = 1:N + 1
    zr = road(t(k));
    ZR(:, k) = zr;
    [~, cor] = corner_states(x, H);
    for c = 1:2
        xc = [cor.zs(c); cor.vs(c); cor.zu(c); cor.vu(c); x(8 + 2 * c - 1); x(8 + 2 * c)];
        switch ctrlType
            case {'afc', 'afc_aw'}
                [Uraw(c, k), Cs{c}] = qc_controller(Cs{c}, t(k), xc, Pq, zr(c), 0);
            case 'skyhook'
                Uraw(c, k) = -2.7 * (xc(5) - a.kappa * (-4000 * xc(2) - 1e5 * xc(1)) / a.A);
            otherwise
                Uraw(c, k) = 0;
        end
    end
    U(:, k) = min(max(Uraw(:, k), -Pq.sim.Vmax), Pq.sim.Vmax);
    dx = hc_rhs(x, U(:, k), zr, H, m, a, passive);
    ACC(:, k) = [dx(2); dx(4)];
    if k == N + 1, break; end
    for j = 1:sub
        tj = t(k) + (j - 1) * h;
        k1 = hc_rhs(x, U(:, k), road(tj), H, m, a, passive);
        k2 = hc_rhs(x + h / 2 * k1, U(:, k), road(tj + h / 2), H, m, a, passive);
        k3 = hc_rhs(x + h / 2 * k2, U(:, k), road(tj + h / 2), H, m, a, passive);
        k4 = hc_rhs(x + h * k3, U(:, k), road(tj + h), H, m, a, passive);
        x = x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
    end
    X(:, k + 1) = x;
end
run.t = t; run.x = X; run.u = U; run.u_raw = Uraw; run.zr = ZR; run.H = H;
run.heave_acc = ACC(1, :)'; run.pitch_acc = ACC(2, :)';
run.zsf = X(1, :)' - H.a * X(3, :)';
run.zsr = X(1, :)' + H.b * X(3, :)';
E = qc_ctrl('afc', 'A');
mu1 = (E.mu0(1) - E.muinf(1)) * exp(-E.alpha(1) * t) + E.muinf(1);
run.mu1 = mu1;
run.metrics = struct('heave_rms', sqrt(mean(run.heave_acc .^ 2)), ...
    'pitch_acc_rms', sqrt(mean(run.pitch_acc .^ 2)), ...
    'pitch_max_deg', max(abs(X(3, :))) * 180 / pi, ...
    'peak_front', max(abs(run.zsf)), 'peak_rear', max(abs(run.zsr)), ...
    'viol_front', max([0; abs(run.zsf) - mu1]), 'viol_rear', max([0; abs(run.zsr) - mu1]), ...
    'sat', mean(abs(Uraw(:)) > Pq.sim.Vmax), 'u_rms', sqrt(mean(U(:) .^ 2)));
run.ctrl = ctrlType; run.road = roadKind;
end

function zr = road_pair(R, t, delay)
[zf, ~] = R.fun(t);
[zb, ~] = R.fun(t - delay);
if strcmp(R.kind, 'case') && t < delay
    zb = 0;      % rear wheel starts on flat road until the front profile reaches it
end
zr = [zf; zb];
end

function [x, cor] = corner_states(x, H)
cor.zs = [x(1) - H.a * x(3); x(1) + H.b * x(3)];
cor.vs = [x(2) - H.a * x(4); x(2) + H.b * x(4)];
cor.zu = [x(5); x(7)];
cor.vu = [x(6); x(8)];
end

function dx = hc_rhs(x, u, zr, H, m, a, passive)
[~, cor] = corner_states(x, H);
dx = zeros(12, 1);
Fb = zeros(2, 1);        % net upward force on the body at each corner
for c = 1:2
    d = cor.zs(c) - cor.zu(c);
    v = cor.vs(c) - cor.vu(c);
    Fs = m.ks * d + m.ksn * d ^ 3;
    Fd = m.bs * v;
    i5 = 8 + 2 * c - 1;
    if passive
        F = 0;
    else
        F = a.A * x(i5) / a.kappa;
        PL = x(i5) / a.kappa;
        xv = x(i5 + 1);
        dx(i5 + 1) = (a.Kv * u(c) - xv) / a.tau;
        dPL = -a.beta * PL - a.alpha * a.A * v + a.gamma * xv * sqrt(max(a.Ps - sign(xv) * PL, 0));
        dx(i5) = a.kappa * dPL;
    end
    Fb(c) = -Fs - Fd + F;
    iu = 5 + 2 * (c - 1);
    dx(iu) = x(iu + 1);
    dx(iu + 1) = (Fs + Fd - m.kt * (cor.zu(c) - zr(c)) - F) / m.mu;
end
dx(1) = x(2);
dx(2) = (Fb(1) + Fb(2)) / H.M;
dx(3) = x(4);
dx(4) = (-H.a * Fb(1) + H.b * Fb(2)) / H.I;
end
