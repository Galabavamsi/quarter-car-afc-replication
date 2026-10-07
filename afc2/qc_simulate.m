function run = qc_simulate(P, R, C, opts)
%QC_SIMULATE Sampled-data closed-loop simulation (50 Hz ZOH controller,
%   RK4 plant integration with step P.sim.h inside each sample).
%
%   run = qc_simulate(P, R, C)
%   run = qc_simulate(P, R, C, opts) with optional fields
%       opts.T         simulation horizon (default P.sim.T)
%       opts.envelope  controller struct whose mu_1(t) is used to audit the
%                      prescribed bound for non-AFC controllers
%                      (default: AFC with the same Case letter)
%       opts.Vmax      override the voltage rail (Inf = ideal unsaturated law)
%       opts.sensing   what the controller sees (default: exact states)
%           .mode  'ideal'  exact states
%                  'diff'   noisy z_s; velocity by differencing z_s;
%                           pressure transducer for x5
%                  'accint' noisy z_s; velocity by leaky integration of a
%                           noisy accelerometer (the paper's choice);
%                           pressure transducer for x5
%                  'kalman' noisy z_s and accelerometer only -> Kalman
%                           filter estimates every state (stands in for
%                           the paper's ESO force estimate)
%           .sig_z (m, default 0.5e-3)  .sig_a (m/s^2, 0.05)
%           .sig_p (Pa, 5e4)  .tau_hp (s, 2)  .seed (1)  .KF (optional)
%
%   run fields: t, x (6xN+1), u (applied), u_raw, acc, F, zr, mu1,
%               zeta/eps/mu (3xN+1, AFC only), diverged, metrics
if nargin < 4
    opts = struct();
end
T = getopt(opts, 'T', P.sim.T);
Vmax = getopt(opts, 'Vmax', P.sim.Vmax);
Ts = P.sim.Ts;
h = P.sim.h;
sub = max(1, round(Ts / h));
h = Ts / sub;
N = round(T / Ts);
passive = strcmp(C.type, 'passive');

t = (0:N)' * Ts;
x = nan(6, N + 1);
x(:, 1) = P.sim.x0;
u = zeros(N + 1, 1);
uraw = zeros(N + 1, 1);
isAfc = any(strcmp(C.type, {'afc', 'afc_aw'}));
zeta = nan(3, N + 1);
epsv = nan(3, N + 1);
muv = nan(3, N + 1);
rhov = zeros(N + 1, 1);
diverged = false;

% ---- sensing path --------------------------------------------------------
sens = getopt(opts, 'sensing', struct('mode', 'ideal'));
smode = getopt(sens, 'mode', 'ideal');
xm = nan(6, N + 1);              % what the controller saw
if ~strcmp(smode, 'ideal')
    sz = getopt(sens, 'sig_z', 0.5e-3);
    sa = getopt(sens, 'sig_a', 0.05);
    sp = getopt(sens, 'sig_p', 5e4);
    seed = getopt(sens, 'seed', 1);
    nz = sz * qc_randn(N + 1, seed);
    na = sa * qc_randn(N + 1, seed + 1000);
    np = sp * qc_randn(N + 1, seed + 2000);
    lam = exp(-Ts / getopt(sens, 'tau_hp', 2));
    vInt = 0;
    zPrev = 0;
    if strcmp(smode, 'kalman')
        KF = getopt(sens, 'KF', []);
        if isempty(KF)
            KF = qc_kalman_design(P, struct('z', sz, 'a', sa));
        end
        xh = zeros(KF.n + 1, 1);
    end
end

for k = 1:N
    [zrk, zrdk] = R.fun(t(k));
    if strcmp(smode, 'ideal')
        xc = x(:, k);
    else
        uPrev = 0;
        if k > 1
            uPrev = u(k - 1);
        end
        [~, aTrue] = qc_plant_rhs(P, x(:, k), uPrev, zrk, zrdk, passive);
        zm = x(1, k) + nz(k);
        am = aTrue + na(k);
        xc = x(:, k);
        xc(1) = zm;
        switch smode
            case 'diff'
                if k > 1
                    xc(2) = (zm - zPrev) / Ts;
                else
                    xc(2) = 0;
                end
                xc(5) = x(5, k) + P.act.kappa * np(k);
            case 'accint'
                vInt = lam * vInt + Ts * am;
                xc(2) = vInt;
                xc(5) = x(5, k) + P.act.kappa * np(k);
            case 'kalman'
                xp = KF.Ad * xh + KF.Bd * uPrev;
                yp = KF.C * xp + KF.D * uPrev;
                xh = xp + KF.Mk * ([zm; am] - yp);
                xc = zeros(6, 1);
                xc(1:KF.n) = xh(1:KF.n);
            otherwise
                error('qc_simulate:sensing', 'Unknown sensing mode "%s".', smode);
        end
        zPrev = zm;
    end
    xm(:, k) = xc;
    [uraw(k), C, info] = qc_controller(C, t(k), xc, P, zrk, zrdk);
    if isAfc
        zeta(:, k) = info.zeta;
        epsv(:, k) = info.eps;
        muv(:, k) = info.mu;
    end
    if isfield(info, 'rho')
        rhov(k) = info.rho;
    end
    if ~isfinite(uraw(k))
        uraw(k) = sign(uraw(k)) * 1e6;
    end
    u(k) = min(max(uraw(k), -Vmax), Vmax);

    xk = x(:, k);
    for j = 1:sub
        tj = t(k) + (j - 1) * h;
        k1 = rhs_at(P, R, xk, u(k), tj, passive);
        k2 = rhs_at(P, R, xk + h / 2 * k1, u(k), tj + h / 2, passive);
        k3 = rhs_at(P, R, xk + h / 2 * k2, u(k), tj + h / 2, passive);
        k4 = rhs_at(P, R, xk + h * k3, u(k), tj + h, passive);
        xk = xk + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
    end
    if any(~isfinite(xk)) || abs(xk(1)) > 10
        diverged = true;
        break;
    end
    x(:, k + 1) = xk;
end
if ~diverged
    u(end) = u(end - 1);
    uraw(end) = uraw(end - 1);
    if isAfc
        [~, info] = qc_afc(C, t(end), x(:, end));
        zeta(:, end) = info.zeta;
        epsv(:, end) = info.eps;
        muv(:, end) = info.mu;
    end
end

acc = nan(N + 1, 1);
F = nan(N + 1, 1);
zr = zeros(N + 1, 1);
for k = 1:N + 1
    [zr(k), zrd] = R.fun(t(k));
    if all(isfinite(x(:, k)))
        [~, acc(k), F(k)] = qc_plant_rhs(P, x(:, k), u(k), zr(k), zrd, passive);
    end
end

% envelope used to audit the prescribed performance bound
if isAfc
    E = C;   % nominal envelope (afc_aw relaxation is not credited)
elseif isfield(opts, 'envelope')
    E = opts.envelope;
else
    v = 'A';
    if isfield(C, 'variant') && strcmp(C.variant, 'B')
        v = 'B';
    end
    E = qc_ctrl('afc', v);
end
mu1 = (E.mu0(1) - E.muinf(1)) * exp(-E.alpha(1) * t) + E.muinf(1);

run.t = t;
run.x = x;
run.u = u;
run.u_raw = uraw;
run.acc = acc;
run.F = F;
run.zr = zr;
run.mu1 = mu1;
run.zeta = zeta;
run.eps = epsv;
run.mu = muv;
run.rho = rhov;
run.x_meas = xm;
run.sensing = smode;
run.diverged = diverged;
run.controller = C;
run.label = C.label;
run.road = R.name;
run.plant = P.name;
run.Vmax = Vmax;
run.metrics = qc_metrics(run, P);
end

function dx = rhs_at(P, R, x, u, t, passive)
[zr, zrd] = R.fun(t);
dx = qc_plant_rhs(P, x, u, zr, zrd, passive);
end

function v = getopt(s, name, default)
if isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default;
end
end
