function S = run_phase1(quick)
%RUN_PHASE1 Tier-1 study: diagnose v1, calibrate a literature plant, compare
%   with the paper's reported numbers, road-sweep hold-out, feasibility map,
%   and local stability of the sampled-data AFC loop.
%
%   S = run_phase1()        full study (about 2-4 min in MATLAB)
%   S = run_phase1(true)    reduced grids for a quick smoke test
%
%   Writes everything to results/phase1/:
%     phase1_summary.txt   human-readable log of every number quoted
%     phase1_results.mat   all runs and tables (struct S)
%     *.csv / *.tex        tables
%     fig_*.pdf / *.png    figures
%
%   Run from the repository root:  S = run_phase1();
if nargin < 1
    quick = false;
end
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'afc2'));
out = fullfile(root, 'results', 'phase1');
if ~exist(out, 'dir')
    mkdir(out);
end
ST = qc_style();
ref = qc_paper_ref();
fid = fopen(fullfile(out, 'phase1_summary.txt'), 'w');
cleaner = onCleanup(@() fclose(fid));
say(fid, 'Phase 1 run started %s (quick=%d)\n', datestr(now), quick);
say(fid, 'Platform: %s\n', platform_string());
tAll = tic;

ctrlNames = {'afc', 'bsc', 'pid', 'passive'};
colors = {ST.afc, ST.bsc, ST.pid, ST.passive};
labels = {'AFC', 'BSC', 'PID', 'Passive'};

%% 1. v1 baseline: reproduce, then diagnose ---------------------------------
say(fid, '\n== 1. v1 baseline (equivalent actuator, Jin et al. masses) ==\n');
P1 = qc_params('v1-equivalent');
R10 = qc_road('case', 10);
v1 = cell(1, 4);
for i = 1:4
    v1{i} = qc_simulate(P1, R10, qc_ctrl(ctrlNames{i}, 'A'));
    m = v1{i}.metrics;
    say(fid, '  %-8s IAE %.4f  ITAE %.4f  ITSE %.4f  accRMS %.4f  sat %.1f%%  PPFviol %.2f mm\n', ...
        labels{i}, m.IAE, m.ITAE, m.ITSE, m.acc_rms, 100 * m.sat_frac, 1e3 * m.ppf_viol);
end
v1ideal = qc_simulate(P1, R10, qc_ctrl('afc', 'A'), struct('Vmax', Inf));
say(fid, '  AFC without the +/-5 V rail: IAE %.4f, max|zs| %.3f m, max|u| %.0f V\n', ...
    v1ideal.metrics.IAE, max(abs(v1ideal.x(1, :))), max(abs(v1ideal.u)));
L1 = qc_linearize(P1, qc_ctrl('afc', 'A'));
say(fid, '  Linearised 50 Hz loop at origin: spectral radius %.4f (stable < 1)\n', L1.rho);
say(fid, '  Equivalent linear AFC law: u = %.1f zs %+.3f zsdot %+.2f x5\n', L1.K(1), L1.K(2), L1.K(5));
say(fid, '  Continuous-equivalent poles: %s\n', fmt_complex(L1.eig_c));
buList = [0.25 0.5 0.75 1 2 5 10 20 50];
rhoBu = zeros(size(buList));
for i = 1:numel(buList)
    Pb = P1;
    Pb.act.bu = buList(i);
    Lb = qc_linearize(Pb, qc_ctrl('afc', 'A'));
    rhoBu(i) = Lb.rho;
end
say(fid, '  rho vs bu: %s\n', sprintf('bu=%g:%.3f  ', [buList; rhoBu]));
S.v1.runs = v1;
S.v1.ideal = v1ideal;
S.v1.lin = L1;
S.v1.bu_list = buList;
S.v1.rho_bu = rhoBu;

%% 2. calibrate the two free scale factors of the literature plant -------------
say(fid, '\n== 2. Calibration of the Alleyne-Hedrick plant to the paper''s Case 10 AFC ==\n');
if quick
    KvList = [3 5 8] * 1e-4;
    tauList = [0.003 1/30];
else
    KvList = [1 2 3 4 5 6 8] * 1e-4;
    tauList = [0.003 0.01 0.02 1/30 0.05];
end
tgt = [ref.IAE(1), ref.ITAE(1), ref.ITSE(1), ref.u_rms_approx(1)];
J = inf(numel(tauList), numel(KvList));
G.IAE = nan(size(J)); G.u_rms = nan(size(J)); G.sat = nan(size(J)); G.viol = nan(size(J));
rows = {};
for a = 1:numel(tauList)
    for b = 1:numel(KvList)
        Pg = qc_params('alleyne');
        Pg.act.tau = tauList(a);
        Pg.act.Kv = KvList(b);
        r = qc_simulate(Pg, R10, qc_ctrl('afc', 'A'));
        m = r.metrics;
        if ~m.diverged
            J(a, b) = mean(abs(log([m.IAE, m.ITAE, m.ITSE, m.u_rms] ./ tgt)));
        end
        G.IAE(a, b) = m.IAE; G.u_rms(a, b) = m.u_rms;
        G.sat(a, b) = m.sat_frac; G.viol(a, b) = m.ppf_viol;
        rows(end + 1, :) = {tauList(a), KvList(b), m.IAE, m.ITAE, m.ITSE, m.u_rms, ...
            m.u_max, m.sat_frac, m.ppf_viol, J(a, b)}; %#ok<AGROW>
    end
end
qc_write_csv(fullfile(out, 'calibration_grid.csv'), ...
    {'tau_s', 'Kv_m_per_V', 'IAE', 'ITAE', 'ITSE', 'u_rms', 'u_max', 'sat_frac', 'ppf_viol_m', 'J'}, rows);
[Jmin, idx] = min(J(:));
[ia, ib] = ind2sub(size(J), idx);
Pc = qc_params('alleyne-cal');
say(fid, '  Targets: IAE %.4f, ITAE %.4f, ITSE %.4f (Fig. 7), u_rms %.2f V (Fig. 13, approx.)\n', tgt);
say(fid, '  Best grid point: tau = %.4f s, Kv = %.1e m/V, J = %.4f (mean |log ratio|)\n', ...
    tauList(ia), KvList(ib), Jmin);
say(fid, '  qc_params(''alleyne-cal'') uses tau = %.4f s, Kv = %.1e m/V\n', Pc.act.tau, Pc.act.Kv);
if abs(tauList(ia) - Pc.act.tau) > 1e-6 || abs(KvList(ib) - Pc.act.Kv) > 1e-12
    say(fid, '  NOTE: grid optimum differs from the stored calibration (expected only in quick mode).\n');
end
jLit = J(abs(tauList - 0.003) < 1e-9, :);
say(fid, '  With the literature spool constant tau = 0.003 s the best J is %.3f\n', min(jLit));
S.cal.KvList = KvList; S.cal.tauList = tauList; S.cal.J = J; S.cal.grid = G;

%% 3. Case 10: AFC/BSC/PID/passive on the calibrated plant vs paper --------------
say(fid, '\n== 3. Case 10 comparison on the calibrated plant (paper gains, unchanged) ==\n');
c10 = cell(1, 4);
rows = {};
for i = 1:4
    c10{i} = qc_simulate(Pc, R10, qc_ctrl(ctrlNames{i}, 'A'));
    m = c10{i}.metrics;
    if i <= 3
        pr = [ref.IAE(i), ref.ITAE(i), ref.ITSE(i), ref.acc_rms(i), ref.acc_max(i), ref.u_rms_approx(i), ref.u_max_approx(i)];
    else
        pr = nan(1, 7);
    end
    rows(end + 1, :) = {labels{i}, m.IAE, pr(1), m.ITAE, pr(2), m.ITSE, pr(3), ...
        m.acc_rms, pr(4), m.acc_max, pr(5), m.u_rms, pr(6), m.u_max, pr(7), ...
        m.sat_frac, m.ppf_viol, m.travel_max, m.tire_ratio_max}; %#ok<AGROW>
    if i <= 3
        say(fid, '  %-8s IAE %.4f (paper %.4f)  ITAE %.3f (%.3f)  ITSE %.4f (%.4f)  accRMS %.3f (%.3f)  uRMS %.2f (~%.1f)  sat %.0f%%  PPFviol %.1f mm\n', ...
            labels{i}, m.IAE, pr(1), m.ITAE, pr(2), m.ITSE, pr(3), m.acc_rms, pr(4), m.u_rms, pr(6), 100 * m.sat_frac, 1e3 * m.ppf_viol);
    else
        say(fid, '  %-8s IAE %.4f  ITAE %.3f  ITSE %.4f  accRMS %.3f  PPFviol %.1f mm\n', ...
            labels{i}, m.IAE, m.ITAE, m.ITSE, m.acc_rms, 1e3 * m.ppf_viol);
    end
end
qc_write_csv(fullfile(out, 'case10_comparison.csv'), ...
    {'controller', 'IAE_sim', 'IAE_paper', 'ITAE_sim', 'ITAE_paper', 'ITSE_sim', 'ITSE_paper', ...
    'accRMS_sim', 'accRMS_paper', 'accMAX_sim', 'accMAX_paper', 'uRMS_sim', 'uRMS_paper_approx', ...
    'uMAX_sim', 'uMAX_paper_approx', 'sat_frac', 'ppf_viol_m', 'travel_max_m', 'tire_load_ratio_max'}, rows);
write_case10_tex(fullfile(out, 'case10_table.tex'), rows);
iae = cellfun(@(r) r.metrics.IAE, c10(1:3));
[~, order] = sort(iae);
say(fid, '  Simulated IAE ranking: %s   (paper: AFC < PID < BSC)\n', strjoin(labels(order), ' < '));
S.case10 = c10;

plot_case10(c10, labels, colors, ST, ref, out);

%% 4. Road sweep: the paper's 8 roads, paper gains per road (hold-out) -----------
say(fid, '\n== 4. Road sweep (Case B gains for 3-9 peaks, Case A for 10) ==\n');
peaks = 3:10;
if quick
    peaks = [3 6 10];
end
names3 = {'afc', 'bsc', 'pid'};
SW = struct();
fields = {'IAE', 'ITAE', 'ITSE', 'u_rms', 'u_max', 'sat_frac', 'ppf_viol'};
for f = 1:numel(fields)
    SW.(fields{f}) = nan(4, numel(peaks));
end
rows = {};
for j = 1:numel(peaks)
    R = qc_road('case', peaks(j));
    variant = 'B';
    if peaks(j) == 10
        variant = 'A';
    end
    for i = 1:4
        if i <= 3
            C = qc_ctrl(names3{i}, variant);
        elseif i == 4
            C = qc_ctrl('afc', 'A');   % row 4: AFC Case A held fixed (v1 protocol)
        end
        r = qc_simulate(Pc, R, C);
        for f = 1:numel(fields)
            SW.(fields{f})(i, j) = r.metrics.(fields{f});
        end
        rows(end + 1, :) = {peaks(j), C.label, r.metrics.IAE, r.metrics.ITAE, r.metrics.ITSE, ...
            r.metrics.u_rms, r.metrics.u_max, r.metrics.sat_frac, r.metrics.ppf_viol}; %#ok<AGROW>
    end
    say(fid, '  %2d peaks: IAE AFC %.3f  BSC %.3f  PID %.3f | AFC(A fixed) %.3f | uRMS AFC %.2f BSC %.2f PID %.2f\n', ...
        peaks(j), SW.IAE(1, j), SW.IAE(2, j), SW.IAE(3, j), SW.IAE(4, j), SW.u_rms(1, j), SW.u_rms(2, j), SW.u_rms(3, j));
end
qc_write_csv(fullfile(out, 'road_sweep.csv'), ...
    {'peaks', 'controller', 'IAE', 'ITAE', 'ITSE', 'u_rms', 'u_max', 'sat_frac', 'ppf_viol_m'}, rows);
afcBest = all(SW.IAE(1, :) <= min(SW.IAE(2:3, :), [], 1));
say(fid, '  AFC has the lowest IAE on %d of %d roads (paper: all 8)\n', ...
    sum(SW.IAE(1, :) <= min(SW.IAE(2:3, :), [], 1)), numel(peaks));
say(fid, '  AFC lowest everywhere: %d\n', afcBest);
SW.peaks = peaks;

% 4b. hold-out test of a jointly fitted plant (train: 10/A and 6/B)
say(fid, '\n  -- 4b. Joint fit (train roads 10/A and 6/B, test the other six) --\n');
Pj = qc_params('alleyne-joint');
SW.joint.IAE = nan(1, numel(peaks));
SW.joint.u_rms = nan(1, numel(peaks));
errTrain = [];
errTest = [];
for j = 1:numel(peaks)
    variant = 'B';
    if peaks(j) == 10
        variant = 'A';
    end
    r = qc_simulate(Pj, qc_road('case', peaks(j)), qc_ctrl('afc', variant));
    SW.joint.IAE(j) = r.metrics.IAE;
    SW.joint.u_rms(j) = r.metrics.u_rms;
    k = find(ref.sweep.peaks == peaks(j));
    e = mean(abs(log([r.metrics.IAE, r.metrics.u_rms] ./ [ref.sweep.IAE(1, k), ref.sweep.u_rms(1, k)])));
    isTrain = any(peaks(j) == [6 10]);
    if isTrain
        errTrain(end + 1) = e; %#ok<AGROW>
    else
        errTest(end + 1) = e; %#ok<AGROW>
    end
    tag = 'test ';
    if isTrain
        tag = 'TRAIN';
    end
    say(fid, '   %s %2d peaks: AFC IAE %.3f (paper ~%.3f)  uRMS %.2f V (paper ~%.2f)\n', ...
        tag, peaks(j), r.metrics.IAE, ref.sweep.IAE(1, k), r.metrics.u_rms, ref.sweep.u_rms(1, k));
end
say(fid, '   mean |log error|: train %.3f, test %.3f\n', mean(errTrain), mean(errTest));
S.sweep = SW;
plot_sweep(SW, ref, ST, out);
plot_holdout(SW, ref, ST, out);

%% 5. Feasibility map: where can the AFC keep its prescribed bound? ---------------
say(fid, '\n== 5. Feasibility maps (AFC Case A) ==\n');
if quick
    fList = [0.2 0.505 1.0];
    sList = [0.5 1 2];
    buList2 = [0.75 5];
else
    fList = [0.188 0.3 0.4 0.505 0.7 1.0 1.5];
    sList = [0.25 0.5 1 2 4];
    buList2 = [0.75 2 5 10 20];
end
FM.viol = nan(numel(sList), numel(fList));
FM.sat = FM.viol;
FM.IAE = FM.viol;
for a = 1:numel(sList)
    for b = 1:numel(fList)
        Pf = Pc;
        Pf.act.Kv = Pc.act.Kv * sList(a);
        r = qc_simulate(Pf, qc_road('sine', 0.043, fList(b)), qc_ctrl('afc', 'A'));
        FM.viol(a, b) = r.metrics.ppf_viol;
        FM.sat(a, b) = r.metrics.sat_frac;
        FM.IAE(a, b) = r.metrics.IAE;
    end
end
FV.viol = nan(numel(buList2), numel(fList));
FV.sat = FV.viol;
for a = 1:numel(buList2)
    for b = 1:numel(fList)
        Pf = P1;
        Pf.act.bu = buList2(a);
        r = qc_simulate(Pf, qc_road('sine', 0.043, fList(b)), qc_ctrl('afc', 'A'));
        FV.viol(a, b) = r.metrics.ppf_viol;
        FV.sat(a, b) = r.metrics.sat_frac;
    end
end
FM.fList = fList; FM.sList = sList; FV.fList = fList; FV.buList = buList2;
say(fid, '  Calibrated plant: bound kept on %d of %d (valve-gain x frequency) cells\n', ...
    nnz(FM.viol < 1e-6), numel(FM.viol));
say(fid, '  v1 plant:         bound kept on %d of %d (bu x frequency) cells\n', ...
    nnz(FV.viol < 1e-6), numel(FV.viol));
S.feas.cal = FM;
S.feas.v1 = FV;
plot_feasibility(FM, FV, out);

%% 6. Local stability of the sampled AFC loop on the calibrated plant -------------
say(fid, '\n== 6. Local stability of the sampled-data AFC loop (calibrated plant) ==\n');
tauGrid = [0.003 0.01 0.02 1/30 0.05 0.1];
rhoTau = zeros(size(tauGrid));
for i = 1:numel(tauGrid)
    Pt = Pc;
    Pt.act.tau = tauGrid(i);
    Lt = qc_linearize(Pt, qc_ctrl('afc', 'A'));
    rhoTau(i) = Lt.rho;
end
TsGrid = [0.02 0.01 0.005 0.002 0.001];
rhoTs = zeros(size(TsGrid));
for i = 1:numel(TsGrid)
    Pt = Pc;
    Pt.sim.Ts = TsGrid(i);
    Pt.sim.h = min(Pc.sim.h, TsGrid(i));
    Lt = qc_linearize(Pt, qc_ctrl('afc', 'A'));
    rhoTs(i) = Lt.rho ^ (0.02 / TsGrid(i));   % per-20-ms growth factor
end
Lc = qc_linearize(Pc, qc_ctrl('afc', 'A'));
say(fid, '  Calibrated loop: rho = %.4f, poles %s\n', Lc.rho, fmt_complex(Lc.eig_c));
say(fid, '  rho vs tau: %s\n', sprintf('%.3f:%.3f  ', [tauGrid; rhoTau]));
say(fid, '  growth per 20 ms vs controller period Ts: %s\n', sprintf('%.3f:%.3f  ', [TsGrid; rhoTs]));
Plc = Pc;
Plc.sim.x0 = [0.001; 0; 0; 0; 0; 0];
lc = qc_simulate(Plc, qc_road('sine', 0, 1), qc_ctrl('afc', 'A'));
tail = lc.t >= 15;
[fq, Pq] = qc_psd(lc.acc(lc.t >= 10), 1 / Pc.sim.Ts);
[~, iq] = max(Pq);
say(fid, '  Zero road, 1 mm initial offset: last-5 s max|zs| = %.3g mm, u_rms = %.2f V, acc_rms = %.3f m/s^2 at %.1f Hz\n', ...
    1e3 * max(abs(lc.x(1, tail))), sqrt(mean(lc.u(tail) .^ 2)), sqrt(mean(lc.acc(tail) .^ 2)), fq(iq));
S.stab.tauGrid = tauGrid; S.stab.rhoTau = rhoTau; S.stab.TsGrid = TsGrid; S.stab.rhoTs = rhoTs;
S.stab.lin = Lc; S.stab.limitcycle = lc;
plot_stability(L1, Lc, v1, v1ideal, S.stab, ST, out);

%% save -----------------------------------------------------------------------
S.params.v1 = P1;
S.params.cal = Pc;
S.ref = ref;
save(fullfile(out, 'phase1_results.mat'), 'S', '-v7');
say(fid, '\nDone in %.1f s. Outputs in %s\n', toc(tAll), out);
end

% ======================================================================
function say(fid, fmt, varargin)
fprintf(fmt, varargin{:});
fprintf(fid, fmt, varargin{:});
end

function s = platform_string()
if exist('OCTAVE_VERSION', 'builtin') ~= 0
    s = ['GNU Octave ', OCTAVE_VERSION];
else
    s = ['MATLAB ', version];
end
end

function s = fmt_complex(z)
z = z(:);
[~, o] = sort(-real(z));
z = z(o);
parts = cell(1, numel(z));
for i = 1:numel(z)
    parts{i} = sprintf('%.2f%+.2fj', real(z(i)), imag(z(i)));
end
s = strjoin(parts, ', ');
end

function fig = newfig(w, h)
fig = figure('Visible', 'off', 'Color', 'w', 'Units', 'centimeters', ...
    'Position', [2 2 w h], 'PaperUnits', 'centimeters', 'PaperSize', [w h], ...
    'PaperPosition', [0 0 w h]);
end

function write_case10_tex(path, rows)
fid = fopen(path, 'w');
c = onCleanup(@() fclose(fid));
fprintf(fid, '%% Generated by run_phase1.m -- calibrated plant, paper Case A gains\n');
fprintf(fid, '\\begin{tabular}{lrrrrrrrr}\n\\toprule\n');
fprintf(fid, ' & \\multicolumn{2}{c}{IAE (m\\,s)} & \\multicolumn{2}{c}{ITAE (m\\,s$^2$)} & \\multicolumn{2}{c}{ITSE (m$^2$s$^2$)} & \\multicolumn{2}{c}{$u_{\\mathrm{RMS}}$ (V)}\\\\\n');
fprintf(fid, 'Controller & sim & paper & sim & paper & sim & paper & sim & paper$^\\ast$\\\\\n\\midrule\n');
for i = 1:size(rows, 1)
    r = rows(i, :);
    fprintf(fid, '%s & %.4f & %s & %.3f & %s & %.4f & %s & %.2f & %s\\\\\n', r{1}, r{2}, num_or_dash(r{3}, '%.4f'), ...
        r{4}, num_or_dash(r{5}, '%.3f'), r{6}, num_or_dash(r{7}, '%.4f'), r{12}, num_or_dash(r{13}, '%.1f'));
end
fprintf(fid, '\\bottomrule\n\\end{tabular}\n');
end

function s = num_or_dash(v, fmt)
if isnan(v)
    s = '--';
else
    s = sprintf(fmt, v);
end
end

function plot_case10(c10, labels, colors, ST, ref, out)
% Fig. 6 analogue: displacement and prescribed bound
fig = newfig(17, 11);
subplot(2, 1, 1); hold on; box on; grid on;
t = c10{1}.t;
plot(t, c10{1}.mu1, '--', 'Color', ST.bound, 'LineWidth', 1);
plot(t, -c10{1}.mu1, '--', 'Color', ST.bound, 'LineWidth', 1);
h = zeros(1, 4);
for i = [4 2 3 1]
    h(i) = plot(c10{i}.t, c10{i}.x(1, :), 'Color', colors{i}, 'LineWidth', ST.lw);
end
ylim([-0.2 0.2]); xlim([0 10]);
ylabel('z_s (m)');
legend(h, labels, 'Location', 'northeast', 'Orientation', 'horizontal');
title('Body displacement against the Case A prescribed bound (calibrated plant)');
subplot(2, 1, 2); hold on; box on; grid on;
plot(t, 1e3 * c10{1}.mu1, '--', 'Color', ST.bound, 'LineWidth', 1);
plot(t, -1e3 * c10{1}.mu1, '--', 'Color', ST.bound, 'LineWidth', 1);
for i = [4 2 3 1]
    plot(c10{i}.t, 1e3 * c10{i}.x(1, :), 'Color', colors{i}, 'LineWidth', ST.lw);
end
xlim([2 10]); ylim([-60 60]);
xlabel('Time (s)'); ylabel('z_s (mm), zoom');
qc_save_fig(fig, fullfile(out, 'fig_case10_displacement'));
close(fig);

% Fig. 8 analogue: acceleration and RMS/MAX against the paper
fig = newfig(17, 11);
subplot(2, 1, 1); hold on; box on; grid on;
for i = [2 3 1]
    plot(c10{i}.t, c10{i}.acc, 'Color', colors{i}, 'LineWidth', 0.9);
end
xlim([0 10]); ylabel('Body accel. (m/s^2)'); xlabel('Time (s)');
legend(labels([2 3 1]), 'Location', 'northeast', 'Orientation', 'horizontal');
title('Sprung-mass acceleration (simulation)');
subplot(2, 2, 3); hold on; box on; grid on;
simv = cellfun(@(r) r.metrics.acc_rms, c10(1:3));
bar([simv(:), ref.acc_rms(:)]);
set(gca, 'XTick', 1:3, 'XTickLabel', labels(1:3), 'YScale', 'log');
ylabel('acc RMS (m/s^2)'); legend({'simulation', 'paper (hardware)'}, 'Location', 'northwest');
subplot(2, 2, 4); hold on; box on; grid on;
simv = cellfun(@(r) r.metrics.acc_max, c10(1:3));
bar([simv(:), ref.acc_max(:)]);
set(gca, 'XTick', 1:3, 'XTickLabel', labels(1:3), 'YScale', 'log');
ylabel('acc MAX (m/s^2)');
qc_save_fig(fig, fullfile(out, 'fig_case10_acceleration'));
close(fig);

% Fig. 9 analogue: acceleration PSD and band energies
fig = newfig(17, 11);
subplot(2, 1, 1); hold on; box on; grid on;
for i = [4 2 3 1]
    [f, Pxx] = qc_psd(c10{i}.acc, 50);
    semilogy(f, Pxx, 'Color', colors{i}, 'LineWidth', 1);
end
set(gca, 'YScale', 'log');
yl = ylim;
plot([4 4], yl, 'k:'); plot([8 8], yl, 'k:');
xlim([0 25]); xlabel('Frequency (Hz)'); ylabel('PSD ((m/s^2)^2/Hz)');
legend(labels([4 2 3 1]), 'Location', 'northeast');
title('Acceleration spectrum; dotted lines mark the paper''s 4-8 Hz band');
subplot(2, 1, 2); hold on; box on; grid on;
B = zeros(4, 3);
for i = 1:4
    m = c10{i}.metrics;
    B(i, :) = [m.band_0_4, m.band_4_8, m.band_8_20];
end
bar(B');
set(gca, 'XTick', 1:3, 'XTickLabel', {'0-4 Hz', '4-8 Hz', '8-20 Hz'}, 'YScale', 'log');
ylabel('mean-square acc. in band ((m/s^2)^2)');
legend(labels, 'Location', 'northeast');
qc_save_fig(fig, fullfile(out, 'fig_case10_psd'));
close(fig);

% Fig. 10 analogue: control voltage
fig = newfig(17, 8);
hold on; box on; grid on;
for i = [2 3 1]
    stairs(c10{i}.t, c10{i}.u, 'Color', colors{i}, 'LineWidth', 1);
end
plot([0 10], [5 5], ':', 'Color', ST.bound); plot([0 10], [-5 -5], ':', 'Color', ST.bound);
xlim([0 10]); ylim([-6 6]); xlabel('Time (s)'); ylabel('Valve command u (V)');
legend(labels([2 3 1]), 'Location', 'southeast', 'Orientation', 'horizontal');
title('Applied control voltage (paper Fig. 10 analogue)');
qc_save_fig(fig, fullfile(out, 'fig_case10_control'));
close(fig);
end

function plot_sweep(SW, ref, ST, out)
cols = {ST.afc, ST.bsc, ST.pid};
mk = {'o', '^', 's'};
names = {'AFC', 'BSC', 'PID'};
fig = newfig(17, 16);
flds = {'IAE', 'ITAE', 'ITSE'};
yl = {'IAE (m s)', 'ITAE (m s^2)', 'ITSE (m^2 s^2)'};
for p = 1:3
    subplot(3, 1, p); hold on; box on; grid on;
    hs = zeros(1, 4);
    for i = 1:3
        hs(i) = plot(SW.peaks, SW.(flds{p})(i, :), ['-', mk{i}], 'Color', cols{i}, ...
            'MarkerFaceColor', cols{i}, 'LineWidth', ST.lw);
        [pk, ia] = intersect(ref.sweep.peaks, SW.peaks);
        plot(pk, ref.sweep.(flds{p})(i, ia), ['--', mk{i}], 'Color', cols{i}, 'LineWidth', 0.8);
    end
    hs(4) = plot(NaN, NaN, 'k--');
    ylabel(yl{p});
    if p == 1
        legend(hs, [names, {'paper (dashed, approx.)'}], 'Location', 'northwest', 'Orientation', 'horizontal');
        title('Road sweep: simulation (solid) vs paper Fig. 12 (dashed)');
    end
end
xlabel('Road case (peaks)');
qc_save_fig(fig, fullfile(out, 'fig_sweep_indices'));
close(fig);

fig = newfig(17, 11);
flds = {'u_rms', 'u_max'};
yl = {'RMS voltage (V)', 'MAX voltage (V)'};
for p = 1:2
    subplot(2, 1, p); hold on; box on; grid on;
    for i = 1:3
        plot(SW.peaks, SW.(flds{p})(i, :), ['-', mk{i}], 'Color', cols{i}, 'MarkerFaceColor', cols{i}, 'LineWidth', ST.lw);
        [pk, ia] = intersect(ref.sweep.peaks, SW.peaks);
        plot(pk, ref.sweep.(flds{p})(i, ia), ['--', mk{i}], 'Color', cols{i}, 'LineWidth', 0.8);
    end
    ylim([0 6]); ylabel(yl{p});
    if p == 1
        title('Control effort: simulation (solid) vs paper Fig. 13 (dashed, approx.)');
    end
end
xlabel('Road case (peaks)');
qc_save_fig(fig, fullfile(out, 'fig_sweep_control'));
close(fig);
end

function plot_holdout(SW, ref, ST, out)
[pk, ia] = intersect(ref.sweep.peaks, SW.peaks);
fig = newfig(17, 13);
flds = {'IAE', 'u_rms'};
yl = {'AFC IAE (m s)', 'AFC RMS voltage (V)'};
for p = 1:2
    subplot(2, 1, p); hold on; box on; grid on;
    h1 = plot(pk, ref.sweep.(flds{p})(1, ia), 'k--o', 'LineWidth', 1);
    h2 = plot(SW.peaks, SW.(flds{p})(1, :), '-o', 'Color', ST.afc, 'MarkerFaceColor', ST.afc, 'LineWidth', ST.lw);
    h3 = plot(SW.peaks, SW.(flds{p})(4, :), ':s', 'Color', ST.afc, 'LineWidth', ST.lw);
    h4 = plot(SW.peaks, SW.joint.(flds{p}), '-.^', 'Color', ST.bsc, 'LineWidth', ST.lw);
    ylabel(yl{p});
    if p == 1
        title('Hold-out test across the paper''s eight roads');
    else
        legend([h1 h2 h3 h4], {'paper (approx.)', 'paper gains', ...
            'Case A gains on all roads', 'joint fit (train 6, 10)'}, ...
            'Location', 'southoutside', 'Orientation', 'horizontal');
    end
end
xlabel('Road case (peaks)');
qc_save_fig(fig, fullfile(out, 'fig_sweep_holdout'));
close(fig);
end

function plot_feasibility(FM, FV, out)
fig = newfig(17, 13);
subplot(2, 2, 1);
heat(1e3 * FM.viol, FM.fList, FM.sList, '%.0f');
ylabel('valve gain / calibrated');
title('Calibrated: violation (mm)');
subplot(2, 2, 2);
heat(100 * FM.sat, FM.fList, FM.sList, '%.0f');
title('Calibrated: saturation (%)');
subplot(2, 2, 3);
heat(1e3 * FV.viol, FV.fList, FV.buList, '%.0f');
ylabel('v1 actuator gain b_u');
xlabel('road frequency (Hz), A = 43 mm');
title('v1: violation (mm)');
subplot(2, 2, 4);
heat(100 * FV.sat, FV.fList, FV.buList, '%.0f');
xlabel('road frequency (Hz), A = 43 mm');
title('v1: saturation (%)');
qc_save_fig(fig, fullfile(out, 'fig_feasibility'));
close(fig);
end

function heat(Z, xs, ys, fmt)
imagesc(1:numel(xs), 1:numel(ys), Z);
set(gca, 'YDir', 'normal', 'XTick', 1:numel(xs), 'XTickLabel', arrayfun(@(v) sprintf('%.2g', v), xs, 'UniformOutput', false), ...
    'YTick', 1:numel(ys), 'YTickLabel', arrayfun(@(v) sprintf('%g', v), ys, 'UniformOutput', false));
colormap(gca, flipud(gray(64)) * 0.75 + 0.25);
for i = 1:numel(ys)
    for j = 1:numel(xs)
        text(j, i, sprintf(fmt, Z(i, j)), 'HorizontalAlignment', 'center', 'FontSize', 8);
    end
end
end

function plot_stability(L1, Lc, v1, v1ideal, st, ST, out)
fig = newfig(18, 15);
subplot(2, 2, 1); hold on; box on; axis equal;
th = linspace(0, 2 * pi, 200);
plot(cos(th), sin(th), 'k-', 'LineWidth', 0.8);
plot(real(L1.eig_d), imag(L1.eig_d), 'x', 'Color', ST.pid, 'MarkerSize', 8, 'LineWidth', 1.5);
plot(real(Lc.eig_d), imag(Lc.eig_d), 'o', 'Color', ST.afc, 'MarkerSize', 6, 'LineWidth', 1.2);
legend({'unit circle', 'v1 plant', 'calibrated'}, 'Location', 'northwest');
title('Eigenvalues, 50 Hz AFC loop');
xlim([-1.4 1.4]); ylim([-1.4 1.4]);
subplot(2, 2, 2); hold on; box on; grid on;
semilogx(st.tauGrid, st.rhoTau, '-o', 'Color', ST.afc, 'LineWidth', ST.lw);
set(gca, 'XScale', 'log');
plot([min(st.tauGrid) max(st.tauGrid)], [1 1], 'k--');
xlabel('valve/sensing lag \tau (s)'); ylabel('spectral radius'); title('Lag vs local stability');
subplot(2, 2, 3); hold on; box on; grid on;
plot(v1{1}.t, 1e3 * v1{1}.x(1, :), 'Color', ST.afc, 'LineWidth', ST.lw);
plot(v1ideal.t, 1e3 * v1ideal.x(1, :), 'Color', ST.bsc, 'LineWidth', 0.9);
xlabel('Time (s)'); ylabel('z_s (mm)'); legend({'\pm5 V rail', 'no rail'}, 'Location', 'southeast');
title('v1: no rail is worse');
subplot(2, 2, 4); hold on; box on; grid on;
lc = st.limitcycle;
plot(lc.t, 1e3 * lc.x(1, :), 'Color', ST.afc, 'LineWidth', 0.9);
xlabel('Time (s)'); ylabel('z_s (mm)');
title('No road: bounded oscillation');
qc_save_fig(fig, fullfile(out, 'fig_stability'));
close(fig);
end
