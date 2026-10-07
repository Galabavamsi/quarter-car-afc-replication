function S = run_phase3(quick)
%RUN_PHASE3 Control-theory depth on the calibrated plant (Tier 2).
%   1. Transformed errors and Lyapunov functions: where Lemma 1 holds and
%      exactly where saturation breaks it
%   2. AFC as an equivalent linear law: transmissibility (analytic,
%      continuous) vs the empirical sampled nonlinear response
%   3. Effort/performance trade-off: AFC vs LQR and skyhook families
%   4. Monte Carlo robustness to plant uncertainty
%   5. Transient (bump) and random (ISO 8608) roads, travel and tire load
%   6. Extension: saturation-aware AFC (envelope relaxation) on the v1 plant
%
%   S = run_phase3()      full study (about 5-8 min in MATLAB)
%   S = run_phase3(true)  reduced grids
%   Outputs: results/phase3/ (phase3_summary.txt, *.csv, fig_*.pdf/png)
if nargin < 1
    quick = false;
end
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'afc2'));
out = fullfile(root, 'results', 'phase3');
if ~exist(out, 'dir')
    mkdir(out);
end
ST = qc_style();
fid = fopen(fullfile(out, 'phase3_summary.txt'), 'w');
cleaner = onCleanup(@() fclose(fid));
say(fid, 'Phase 3 run started %s (quick=%d)\n', datestr(now), quick);
tAll = tic;
Pc = qc_params('alleyne-cal');
P1 = qc_params('v1-equivalent');
R10 = qc_road('case', 10);
Cafc = qc_ctrl('afc', 'A');

%% 1. transformed errors / Lyapunov functions ----------------------------------
say(fid, '\n== 1. Transformed errors and V_i = eps_i^2/2 ==\n');
rc = qc_simulate(Pc, R10, Cafc);
rv = qc_simulate(P1, R10, Cafc);
for k = 1:2
    if k == 1
        r = rc; nm = 'calibrated';
    else
        r = rv; nm = 'v1';
    end
    zmax = max(abs(r.zeta), [], 2);
    emax = max(abs(r.eps), [], 2);
    say(fid, '  %-10s max|zeta_i| = [%.3f %.3f %.3f] (must stay < 1), max|eps_i| = [%.2f %.2f %.2f]\n', ...
        nm, zmax, emax);
    sat = abs(r.u_raw) > r.Vmax;
    viol = any(abs(r.zeta) >= 1, 1)';
    say(fid, '  %-10s samples with |zeta|>=1: %.1f%%; of those, saturated: %.1f%%\n', nm, ...
        100 * mean(viol), 100 * mean(sat(viol)));
end
% Lemma 2: |eps| <= epsM  <=>  zeta in [ (e^-2eM - 1)/(e^-2eM + 1), (e^2eM - 1)/(e^2eM + 1) ] = tanh(+-eM)
epsM = max(abs(rc.eps(:, rc.t > 1)), [], 2);
say(fid, '  Lemma 2 check (calibrated, t>1 s): eps_M = [%.3f %.3f %.3f] -> |zeta| <= tanh(eps_M) = [%.3f %.3f %.3f]\n', ...
    epsM, tanh(epsM));
plot_lyapunov(rc, rv, ST, out);
S.lyap.cal = rc;
S.lyap.v1 = rv;

%% 2. equivalent linear law and frequency response --------------------------------
say(fid, '\n== 2. Frequency response ==\n');
Lafc = qc_linearize(Pc, Cafc);
say(fid, '  AFC Case A near the origin is the static law u = K x with K = %s\n', mat2str(Lafc.K, 4));
a = Pc.act;
say(fid, '  i.e. a skyhook spring of %.0f N/m and skyhook damper of %.0f N s/m behind a pressure loop of %.2f V/unit\n', ...
    -Lafc.K(1) / -Lafc.K(5) * a.A / a.kappa, -Lafc.K(2) / -Lafc.K(5) * a.A / a.kappa, -Lafc.K(5));
Clqr = qc_lqr_design(Pc, struct('qz', 1e8, 'qa', 1, 'r', 1));
Csky = qc_ctrl('skyhook');
Lin = qc_lin_cont(Pc);
LinP = qc_lin_cont(Pc, true);
n = Lin.n;
w = 2 * pi * logspace(-1, log10(25), 300);
Ksky = [-Csky.kp * a.kappa * Csky.ksky / a.A, -Csky.kp * a.kappa * Csky.csky / a.A, 0, 0, -Csky.kp, zeros(1, n - 5)];
Ks = {zeros(1, n), Lafc.K(1:n), -Clqr.K, Ksky};
names = {'passive', 'AFC (linearised)', 'LQR', 'Skyhook'};
cols = {ST.passive, ST.afc, ST.pid, ST.bsc};
Hz = zeros(4, numel(w));
Ha = zeros(4, numel(w));
for i = 1:4
    if i == 1
        L = LinP; K = zeros(1, LinP.n);
    else
        L = Lin; K = Ks{i};
    end
    Acl = L.A + L.B * K;
    Ca = L.Ca + L.Da * K;
    for j = 1:numel(w)
        X = (1i * w(j) * eye(L.n) - Acl) \ L.E;
        Hz(i, j) = abs(L.Cz * X);
        Ha(i, j) = abs(Ca * X + L.Ea);
    end
    say(fid, '  %-17s continuous closed-loop max Re(pole) = %.2f\n', names{i}, max(real(eig(Acl))));
end
if quick
    fs = [0.3 1 3 8];
else
    fs = [0.2 0.3 0.5 0.8 1.2 2 3 4 5 6 8 10 12 15 20];
end
Gs = cell(1, 4);
Cs = {qc_ctrl('passive'), Cafc, Clqr, Csky};
for i = 1:4
    Gs{i} = qc_sine_gain(Pc, Cs{i}, fs, 0.005);
end
say(fid, '  Empirical |zs/zr| at 0.5 Hz: passive %.3f, AFC %.3f, LQR %.3f, skyhook %.3f\n', ...
    interp_gain(Gs{1}, 0.5), interp_gain(Gs{2}, 0.5), interp_gain(Gs{3}, 0.5), interp_gain(Gs{4}, 0.5));
band = fs >= 4 & fs <= 8;
for i = 1:4
    say(fid, '  %-17s mean |a_s/z_r| over 4-8 Hz (sampled, nonlinear): %.1f 1/s^2; residual (non-harmonic) zs RMS max %.2g m\n', ...
        names{i}, mean(Gs{i}.acc(band)), max(Gs{i}.resid_zs));
end
plot_freq(w, Hz, Ha, Gs, names, cols, ST, out);
S.freq.w = w; S.freq.Hz = Hz; S.freq.Ha = Ha; S.freq.G = Gs; S.freq.K_afc = Lafc.K;

%% 3. effort vs performance (Case 10) ---------------------------------------------
say(fid, '\n== 3. Effort/performance trade-off (Case 10) ==\n');
fam = {};
if quick
    qzs = [1e6 1e8 1e10];
    skyC = [2000 8000];
    skyK = [0 1e5];
    kscale = [0.5 1];
else
    qzs = [1e5 1e6 1e7 1e8 1e9 1e10];
    skyC = [1000 3000 8000];
    skyK = [0 3e4 1e5];
    kscale = [0.25 0.5 1 2];
end
for q = qzs
    fam(end + 1, :) = {'LQR', qc_lqr_design(Pc, struct('qz', q, 'qa', 1, 'r', 1)), sprintf('q_z=%g', q)}; %#ok<AGROW>
end
for c = skyC
    for k = skyK
        C = qc_ctrl('skyhook'); C.csky = c; C.ksky = k;
        fam(end + 1, :) = {'Skyhook', C, sprintf('c=%g,k=%g', c, k)}; %#ok<AGROW>
    end
end
for s = kscale
    C = Cafc; C.k = C.k * s;
    fam(end + 1, :) = {'AFC', C, sprintf('k x%g', s)}; %#ok<AGROW>
end
fam(end + 1, :) = {'AFC-B', qc_ctrl('afc', 'B'), 'Case B gains'};
fam(end + 1, :) = {'BSC', qc_ctrl('bsc', 'A'), 'Case A'};
fam(end + 1, :) = {'PID', qc_ctrl('pid', 'A'), 'Case A'};
fam(end + 1, :) = {'Passive', qc_ctrl('passive'), ''};
TO = zeros(size(fam, 1), 5);
rows = {};
for i = 1:size(fam, 1)
    r = qc_simulate(Pc, R10, fam{i, 2});
    m = r.metrics;
    TO(i, :) = [m.u_rms, m.IAE, m.acc_rms, m.ppf_viol, m.sat_frac];
    rows(end + 1, :) = {fam{i, 1}, fam{i, 3}, m.u_rms, m.IAE, m.acc_rms, m.ppf_viol, m.sat_frac, m.travel_max}; %#ok<AGROW>
    say(fid, '  %-8s %-14s uRMS %.2f V  IAE %.4f  accRMS %.3f  viol %.1f mm  sat %.0f%%\n', ...
        fam{i, 1}, fam{i, 3}, TO(i, :) .* [1 1 1 1e3 100]);
end
qc_write_csv(fullfile(out, 'tradeoff_case10.csv'), ...
    {'family', 'setting', 'u_rms', 'IAE', 'acc_rms', 'ppf_viol_m', 'sat_frac', 'travel_max_m'}, rows);
plot_tradeoff(fam, TO, ST, out);
S.tradeoff.family = fam(:, [1 3]);
S.tradeoff.TO = TO;

%% 4. Monte Carlo robustness --------------------------------------------------------
say(fid, '\n== 4. Monte Carlo robustness (calibrated plant, Case 10) ==\n');
if quick
    N = 6;
else
    N = 40;
end
U = qc_kronecker(N, 7);
% ranges: ms +-15%, ks +-20%, bs +-30%, kt +-10%, Kv +-30%, tau x[0.5,2], beta x[0.5,2]
mc = {qc_ctrl('afc', 'A'), qc_ctrl('bsc', 'A'), qc_ctrl('pid', 'A'), qc_ctrl('skyhook')};
mcn = {'AFC', 'BSC', 'PID', 'Skyhook'};
MC.IAE = nan(N, 4); MC.u_rms = MC.IAE; MC.viol = MC.IAE; MC.acc = MC.IAE; MC.div = false(N, 4);
for i = 1:N
    Pi = Pc;
    Pi.mech.ms = Pc.mech.ms * (0.85 + 0.30 * U(i, 1));
    Pi.mech.ks = Pc.mech.ks * (0.80 + 0.40 * U(i, 2));
    Pi.mech.bs = Pc.mech.bs * (0.70 + 0.60 * U(i, 3));
    Pi.mech.kt = Pc.mech.kt * (0.90 + 0.20 * U(i, 4));
    Pi.act.Kv = Pc.act.Kv * (0.70 + 0.60 * U(i, 5));
    Pi.act.tau = Pc.act.tau * 2 ^ (2 * U(i, 6) - 1);
    Pi.act.beta = Pc.act.beta * 2 ^ (2 * U(i, 7) - 1);
    for j = 1:4
        C = mc{j};
        r = qc_simulate(Pi, R10, C);   % BSC keeps the NOMINAL model? see note below
        MC.IAE(i, j) = r.metrics.IAE;
        MC.u_rms(i, j) = r.metrics.u_rms;
        MC.viol(i, j) = r.metrics.ppf_viol;
        MC.acc(i, j) = r.metrics.acc_rms;
        MC.div(i, j) = r.diverged;
    end
end
% NOTE: qc_bsc reads the model from the plant struct it is given, so in
% this loop BSC is granted EXACT knowledge of each perturbed plant -- a
% best case for BSC. AFC, PID and skyhook use no model at all.
for j = 1:4
    say(fid, '  %-8s IAE median %.4f [%.4f, %.4f]  bound kept %d/%d  uRMS median %.2f V  diverged %d\n', ...
        mcn{j}, median(MC.IAE(:, j)), min(MC.IAE(:, j)), max(MC.IAE(:, j)), nnz(MC.viol(:, j) < 1e-6), N, ...
        median(MC.u_rms(:, j)), nnz(MC.div(:, j)));
end
say(fid, '  (BSC is given the exact perturbed model: a best case for BSC.)\n');
plot_mc(MC, mcn, ST, out);
S.mc = MC;

%% 5. bump and ISO 8608 roads ------------------------------------------------------
say(fid, '\n== 5. Transient bump and ISO 8608 random road ==\n');
roads = {qc_road('bump', 0.05, 2.5, 20 / 3.6, 3), qc_road('iso8608', 'C', 15, 1)};
Tr = [8, 20];
cset = {qc_ctrl('passive'), qc_ctrl('afc', 'A'), qc_ctrl('afc_aw', 'A'), Csky, Clqr, qc_ctrl('pid', 'A')};
cn = {'Passive', 'AFC', 'AFC-AW', 'Skyhook', 'LQR', 'PID'};
BR = cell(2, numel(cset));
rows = {};
for k = 1:2
    say(fid, '  %s\n', roads{k}.name);
    for j = 1:numel(cset)
        r = qc_simulate(Pc, roads{k}, cset{j}, struct('T', Tr(k)));
        BR{k, j} = r;
        m = r.metrics;
        pk = max(abs(r.x(1, :)));
        rows(end + 1, :) = {roads{k}.name, cn{j}, pk, m.acc_rms, m.acc_max, m.travel_max, m.tire_ratio_max, m.u_rms, m.sat_frac, m.ppf_viol}; %#ok<AGROW>
        say(fid, '    %-8s peak|zs| %.1f mm  accRMS %.3f  accMAX %.2f  travel %.1f mm  tire-load ratio %.2f  uRMS %.2f V  sat %.0f%%\n', ...
            cn{j}, 1e3 * pk, m.acc_rms, m.acc_max, 1e3 * m.travel_max, m.tire_ratio_max, m.u_rms, 100 * m.sat_frac);
    end
end
qc_write_csv(fullfile(out, 'bump_iso_roads.csv'), {'road', 'controller', 'peak_zs_m', 'acc_rms', 'acc_max', ...
    'travel_max_m', 'tire_load_ratio_max', 'u_rms', 'sat_frac', 'ppf_viol_m'}, rows);
plot_roads(BR, cn, roads, ST, out);
S.roads.runs = BR;
S.roads.names = cn;

%% 6. extension: saturation-aware AFC on the saturating v1 plant -----------------------
say(fid, '\n== 6. Extension: AFC with envelope relaxation (anti-windup), v1 plant ==\n');
ra = qc_simulate(P1, R10, qc_ctrl('afc', 'A'));
rw = qc_simulate(P1, R10, qc_ctrl('afc_aw', 'A'));
for k = 1:2
    if k == 1
        r = ra; nm = 'AFC';
    else
        r = rw; nm = 'AFC-AW';
    end
    m = r.metrics;
    du = mean(abs(diff(r.u)));        % command activity, V per sample
    say(fid, '  %-7s IAE %.4f  accRMS %.3f  sat %.0f%%  nominal-bound viol %.1f mm  mean|du| %.3f V/sample\n', ...
        nm, m.IAE, m.acc_rms, 100 * m.sat_frac, 1e3 * m.ppf_viol, du);
end
say(fid, '  max envelope relaxation rho = %.2f (envelope widened by %.0f%%)\n', max(rw.rho), 100 * max(rw.rho));
plot_aw(ra, rw, ST, out);
S.aw.afc = ra;
S.aw.afc_aw = rw;

save(fullfile(out, 'phase3_results.mat'), 'S', '-v7');
say(fid, '\nDone in %.1f s. Outputs in %s\n', toc(tAll), out);
end

% ======================================================================
function say(fid, fmt, varargin)
fprintf(fmt, varargin{:});
fprintf(fid, fmt, varargin{:});
end

function g = interp_gain(G, f)
g = interp1(G.f, G.zs, f, 'linear', 'extrap');
end

function fig = newfig(w, h)
fig = figure('Visible', 'off', 'Color', 'w', 'Units', 'centimeters', ...
    'Position', [2 2 w h], 'PaperUnits', 'centimeters', 'PaperSize', [w h], ...
    'PaperPosition', [0 0 w h]);
end

function plot_lyapunov(rc, rv, ST, out)
fig = newfig(18, 16);
runs = {rc, rv};
ttl = {'calibrated plant', 'v1 plant'};
for c = 1:2
    r = runs{c};
    subplot(3, 2, c); hold on; box on; grid on;
    plot(r.t, r.zeta(1, :), 'Color', ST.afc, 'LineWidth', ST.lw);
    plot(r.t, r.zeta(2, :), 'Color', ST.bsc, 'LineWidth', 0.8);
    plot(r.t, r.zeta(3, :), 'Color', ST.pid, 'LineWidth', 0.8);
    plot([0 r.t(end)], [1 1], '--', 'Color', ST.bound); plot([0 r.t(end)], [-1 -1], '--', 'Color', ST.bound);
    ylim([-2.5 2.5]); ylabel('\zeta_i = e_i/\mu_i');
    title(['Normalised errors, ', ttl{c}]);
    if c == 1
        legend({'\zeta_1', '\zeta_2', '\zeta_3'}, 'Location', 'southeast', 'Orientation', 'horizontal');
    end
    subplot(3, 2, 2 + c); hold on; box on; grid on;
    plot(r.t, r.eps(1, :), 'Color', ST.afc, 'LineWidth', ST.lw);
    plot(r.t, r.eps(2, :), 'Color', ST.bsc, 'LineWidth', 0.8);
    plot(r.t, r.eps(3, :), 'Color', ST.pid, 'LineWidth', 0.8);
    ylabel('\epsilon_i');
    subplot(3, 2, 4 + c); hold on; box on; grid on;
    V = 0.5 * r.eps .^ 2;
    semilogy(r.t, max(V(1, :), 1e-8), 'Color', ST.afc, 'LineWidth', ST.lw);
    set(gca, 'YScale', 'log');
    sat = abs(r.u_raw) > r.Vmax;
    yl = [1e-6 1e3];
    ylim(yl);
    ts = r.t(sat);
    plot(ts, yl(2) * 0.5 * ones(size(ts)), '.', 'Color', ST.bound, 'MarkerSize', 4);
    xlabel('Time (s)'); ylabel('V_1 = \epsilon_1^2/2');
    if c == 2
        title('red dots: valve saturated');
    end
end
qc_save_fig(fig, fullfile(out, 'fig_lyapunov'));
close(fig);
end

function plot_freq(w, Hz, Ha, Gs, names, cols, ST, out)
f = w / (2 * pi);
fig = newfig(17, 14);
subplot(2, 1, 1); hold on; box on; grid on;
h = zeros(1, 4);
for i = 1:4
    h(i) = loglog(f, Hz(i, :), 'Color', cols{i}, 'LineWidth', ST.lw);
    plot(Gs{i}.f, Gs{i}.zs, 'o', 'Color', cols{i}, 'MarkerFaceColor', cols{i}, 'MarkerSize', 4);
end
set(gca, 'XScale', 'log', 'YScale', 'log');
yl = ylim; plot([4 4], yl, 'k:'); plot([8 8], yl, 'k:');
ylabel('|z_s / z_r|');
legend(h, names, 'Location', 'northeast');
title('Road-to-body transmissibility: lines = continuous linearised, dots = sampled nonlinear');
subplot(2, 1, 2); hold on; box on; grid on;
for i = 1:4
    loglog(f, Ha(i, :), 'Color', cols{i}, 'LineWidth', ST.lw);
    plot(Gs{i}.f, Gs{i}.acc, 'o', 'Color', cols{i}, 'MarkerFaceColor', cols{i}, 'MarkerSize', 4);
end
set(gca, 'XScale', 'log', 'YScale', 'log');
yl = ylim; plot([4 4], yl, 'k:'); plot([8 8], yl, 'k:');
xlabel('Frequency (Hz)'); ylabel('|a_s / z_r| (1/s^2)');
qc_save_fig(fig, fullfile(out, 'fig_frequency_response'));
close(fig);
end

function plot_tradeoff(fam, TO, ST, out)
fig = newfig(17, 9);
groups = {'LQR', 'Skyhook', 'AFC', 'AFC-B', 'BSC', 'PID', 'Passive'};
cols = {ST.pid, ST.bsc, ST.afc, ST.afc, [0.6 0.3 0.1], [0.3 0.3 0.5], ST.passive};
mk = {'s-', '^', 'o-', 'p', 'v', 'd', 'x'};
for p = 1:2
    subplot(1, 2, p); hold on; box on; grid on;
    hs = [];
    for g = 1:numel(groups)
        sel = strcmp(fam(:, 1), groups{g});
        if ~any(sel)
            continue;
        end
        y = TO(sel, 1 + p);
        x = TO(sel, 1);
        [x, o] = sort(x);
        y = y(o);
        hs(end + 1) = plot(x, y, mk{g}, 'Color', cols{g}, 'MarkerFaceColor', cols{g}, 'LineWidth', 1); %#ok<AGROW>
    end
    set(gca, 'YScale', 'log');
    xlabel('RMS valve voltage (V)');
    if p == 1
        ylabel('IAE (m s)');
        legend(hs, groups, 'Location', 'northeast');
        title('Displacement vs effort');
    else
        ylabel('acc RMS (m/s^2)');
        title('Comfort vs effort');
    end
end
qc_save_fig(fig, fullfile(out, 'fig_tradeoff'));
close(fig);
end

function plot_mc(MC, mcn, ST, out)
fig = newfig(17, 8);
cols = {ST.afc, ST.bsc, ST.pid, [0.5 0.5 0.2]};
vals = {MC.IAE, 1e3 * MC.viol, MC.u_rms};
yl = {'IAE (m s)', 'bound violation (mm)', 'RMS voltage (V)'};
for p = 1:3
    subplot(1, 3, p); hold on; box on; grid on;
    V = vals{p};
    for j = 1:4
        x = j + 0.25 * (qc_kronecker(size(V, 1), 1) - 0.5);
        plot(x, V(:, j), '.', 'Color', cols{j}, 'MarkerSize', 10);
        plot(j + [-0.3 0.3], median(V(:, j)) * [1 1], 'k-', 'LineWidth', 1.5);
    end
    set(gca, 'XTick', 1:4, 'XTickLabel', mcn);
    ylabel(yl{p});
    if p == 1
        set(gca, 'YScale', 'log');
        title('Monte Carlo (black = median)');
    end
end
qc_save_fig(fig, fullfile(out, 'fig_monte_carlo'));
close(fig);
end

function plot_roads(BR, cn, roads, ST, out)
cols = {ST.passive, ST.afc, [0.2 0.6 0.6], ST.bsc, ST.pid, [0.3 0.3 0.5]};
fig = newfig(18, 15);
for k = 1:2
    subplot(3, 2, k); hold on; box on; grid on;
    plot(BR{k, 1}.t, 1e3 * BR{k, 1}.zr, 'k:', 'LineWidth', 1);
    for j = 1:numel(cn)
        plot(BR{k, j}.t, 1e3 * BR{k, j}.x(1, :), 'Color', cols{j}, 'LineWidth', 1);
    end
    ylabel('z_s (mm)'); title(roads{k}.name);
    if k == 1
        xlim([2.5 6]);
        plot(BR{k, 2}.t, 1e3 * BR{k, 2}.mu1, '--', 'Color', ST.bound);
        plot(BR{k, 2}.t, -1e3 * BR{k, 2}.mu1, '--', 'Color', ST.bound);
        legend([{'road'}, cn], 'Location', 'northeast');
    else
        xlim([0 10]);
    end
    subplot(3, 2, 2 + k); hold on; box on; grid on;
    for j = 1:numel(cn)
        plot(BR{k, j}.t, BR{k, j}.acc, 'Color', cols{j}, 'LineWidth', 0.8);
    end
    ylabel('a_s (m/s^2)');
    if k == 1, xlim([2.5 6]); else, xlim([0 10]); end
    subplot(3, 2, 4 + k); hold on; box on; grid on;
    for j = 2:numel(cn)
        stairs(BR{k, j}.t, BR{k, j}.u, 'Color', cols{j}, 'LineWidth', 0.8);
    end
    ylabel('u (V)'); xlabel('Time (s)'); ylim([-5.5 5.5]);
    if k == 1, xlim([2.5 6]); else, xlim([0 10]); end
end
qc_save_fig(fig, fullfile(out, 'fig_bump_iso'));
close(fig);
end

function plot_aw(ra, rw, ST, out)
fig = newfig(17, 13);
subplot(3, 1, 1); hold on; box on; grid on;
plot(ra.t, 1e3 * ra.mu1, '--', 'Color', ST.bound); plot(ra.t, -1e3 * ra.mu1, '--', 'Color', ST.bound);
plot(ra.t, 1e3 * ra.x(1, :), 'Color', ST.afc, 'LineWidth', ST.lw);
plot(rw.t, 1e3 * rw.x(1, :), 'Color', ST.bsc, 'LineWidth', ST.lw);
ylabel('z_s (mm)'); ylim([-80 80]); legend({'nominal bound', '', 'AFC', 'AFC-AW'}, 'Location', 'northeast', 'Orientation', 'horizontal');
title('v1 plant (weak actuator): envelope relaxation vs bang-bang');
subplot(3, 1, 2); hold on; box on; grid on;
stairs(ra.t, ra.u, 'Color', ST.afc); stairs(rw.t, rw.u, 'Color', ST.bsc);
ylabel('u (V)'); ylim([-5.5 5.5]);
subplot(3, 1, 3); hold on; box on; grid on;
plot(rw.t, rw.rho, 'Color', ST.bsc, 'LineWidth', ST.lw);
ylabel('\rho (relaxation)'); xlabel('Time (s)');
qc_save_fig(fig, fullfile(out, 'fig_antiwindup'));
close(fig);
end
