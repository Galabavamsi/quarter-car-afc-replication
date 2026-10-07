function S = run_phase2(quick)
%RUN_PHASE2 Implementation realism: Simulink v2 cross-check, the paper's
%   sensing path (noisy displacement + accelerometer, velocity by
%   integration, observer for the force state) and control-law timing.
%
%   S = run_phase2()       full study
%   S = run_phase2(true)   reduced noise sweep
%   Section A needs Simulink and is skipped automatically without it.
%   Outputs: results/phase2/ (phase2_summary.txt, *.csv, fig_*.pdf/png)
if nargin < 1
    quick = false;
end
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'afc2'));
out = fullfile(root, 'results', 'phase2');
if ~exist(out, 'dir')
    mkdir(out);
end
ST = qc_style();
fid = fopen(fullfile(out, 'phase2_summary.txt'), 'w');
cleaner = onCleanup(@() fclose(fid));
say(fid, 'Phase 2 run started %s (quick=%d)\n', datestr(now), quick);
tAll = tic;
Pc = qc_params('alleyne-cal');
R10 = qc_road('case', 10);
Cafc = qc_ctrl('afc', 'A');
N = round(Pc.sim.T / Pc.sim.Ts);

%% A. Simulink v2 model and cross-check --------------------------------------
say(fid, '\n== A. Simulink v2 (servo-valve plant, 50 Hz digital controller) ==\n');
hasSimulink = exist('OCTAVE_VERSION', 'builtin') == 0 && ~isempty(ver('simulink')) && license('test', 'Simulink');
if hasSimulink
    for mode = {'ideal', 'accint'}
        model = build_quarter_car_simulink_v2(true, 'alleyne-cal', 'A', mode{1});
        t = (0:N)' * Pc.sim.Ts;
        noise = [t, qc_randn(N + 1, 1), qc_randn(N + 1, 1001), qc_randn(N + 1, 2001)];
        in = Simulink.SimulationInput(model);
        in = in.setVariable('qc_noise', noise);
        so = sim(in);
        xs = so.qc_xlog;
        us = so.qc_u;
        D = squeeze(xs.Data);
        if size(D, 1) == 6 && size(D, 2) ~= 6
            D = D';                      % -> N x 6
        end
        zsS = interp1(xs.Time, D(:, 1), t, 'linear');
        uS = interp1(us.Time, squeeze(us.Data), t, 'previous');
        opts = struct('sensing', struct('mode', mode{1}, 'seed', 1));
        r = qc_simulate(Pc, R10, Cafc, opts);
        dz = max(abs(zsS(:) - r.x(1, :)'));
        du = max(abs(uS(1:end - 1) - r.u(1:end - 1)));
        say(fid, '  sensing=%-6s  max|zs_Simulink - zs_MATLAB| = %.3g m, max|du| = %.3g V\n', mode{1}, dz, du);
        S.simulink.(mode{1}) = struct('dz', dz, 'du', du);
        try
            print(['-s', model], '-dpng', '-r200', fullfile(out, ['simulink_v2_', mode{1}, '.png']));
        catch err
            say(fid, '  (model image export failed: %s)\n', err.message);
        end
    end
    build_quarter_car_simulink_v2(true, 'alleyne-cal', 'A', 'ideal');   % leave the ideal model on disk
    say(fid, '  QuarterCar_AFC_v2.slx rebuilt in ideal-sensing mode.\n');
else
    say(fid, '  Simulink not available here: skipped (run this in MATLAB with Simulink).\n');
end

%% B. sensing path --------------------------------------------------------------
say(fid, '\n== B. Sensing path (AFC Case A, calibrated plant, Case 10) ==\n');
modes = {'ideal', 'diff', 'accint', 'kalman'};
mlab = {'exact states', 'z_s + differencing', 'z_s + integrated accelerometer', 'z_s + accelerometer -> Kalman'};
SB = cell(1, 4);
rows = {};
for i = 1:4
    r = qc_simulate(Pc, R10, Cafc, struct('sensing', struct('mode', modes{i})));
    SB{i} = r;
    m = r.metrics;
    k = 1:N;
    ev = sqrt(mean((r.x_meas(2, k) - r.x(2, k)) .^ 2));
    e5 = sqrt(mean((r.x_meas(5, k) - r.x(5, k)) .^ 2));
    rows(end + 1, :) = {modes{i}, m.IAE, m.ITAE, m.ITSE, m.acc_rms, m.u_rms, m.sat_frac, m.ppf_viol, ev, e5}; %#ok<AGROW>
    say(fid, '  %-34s IAE %.4f  accRMS %.3f  uRMS %.2f V  sat %.0f%%  viol %.1f mm | vel. error %.4f m/s, x5 error %.3f\n', ...
        mlab{i}, m.IAE, m.acc_rms, m.u_rms, 100 * m.sat_frac, 1e3 * m.ppf_viol, ev, e5);
end
qc_write_csv(fullfile(out, 'sensing_modes.csv'), {'mode', 'IAE', 'ITAE', 'ITSE', 'acc_rms', 'u_rms', ...
    'sat_frac', 'ppf_viol_m', 'vel_err_rms', 'x5_err_rms'}, rows);
if quick
    scales = [0 1 4];
else
    scales = [0 0.5 1 2 4 8];
end
NS.scales = scales;
NS.IAE = nan(3, numel(scales)); NS.acc = NS.IAE; NS.u = NS.IAE; NS.ev = NS.IAE;
sm = {'diff', 'accint', 'kalman'};
for j = 1:numel(scales)
    s = max(scales(j), 1e-6);
    for i = 1:3
        sens = struct('mode', sm{i}, 'sig_z', 0.5e-3 * s, 'sig_a', 0.05 * s, 'sig_p', 5e4 * s);
        r = qc_simulate(Pc, R10, Cafc, struct('sensing', sens));
        NS.IAE(i, j) = r.metrics.IAE;
        NS.acc(i, j) = r.metrics.acc_rms;
        NS.u(i, j) = r.metrics.u_rms;
        NS.ev(i, j) = sqrt(mean((r.x_meas(2, 1:N) - r.x(2, 1:N)) .^ 2));
    end
    say(fid, '  noise x%-4g vel. error: diff %.4f, accint %.4f, kalman %.4f m/s | accRMS: %.3f / %.3f / %.3f\n', ...
        scales(j), NS.ev(:, j), NS.acc(:, j));
end
say(fid, '  Paper''s claim (Sec. IV): integrating the accelerometer beats differencing z_s for velocity.\n');
say(fid, '  At nominal noise: differencing %.4f vs integration %.4f m/s RMS error.\n', ...
    NS.ev(1, scales == 1), NS.ev(2, scales == 1));
S.sensing.modes = SB;
S.sensing.sweep = NS;
plot_sensing(SB, NS, mlab, ST, out);

%% C. control-law timing ------------------------------------------------------
say(fid, '\n== C. Control-law cost (paper Table III: AFC 4.2, BSC 5.8, PID 2.6 ms) ==\n');
x = [0.005; 0.02; 0.01; -0.03; 0.3; 1e-4];
Cb = qc_ctrl('bsc', 'A');
Cp = qc_ctrl('pid', 'A');
reps = 20000;
if exist('OCTAVE_VERSION', 'builtin') ~= 0
    reps = 2000;
end
tt = zeros(1, 3);
tic; for i = 1:reps, u = qc_afc(Cafc, 0.02 * i, x); end; tt(1) = toc / reps; %#ok<NASGU>
tic; for i = 1:reps, u = qc_bsc(Cb, x, Pc, 0.01, 0); end; tt(2) = toc / reps; %#ok<NASGU>
tic; for i = 1:reps, [u, Cp] = qc_controller(Cp, 0, x, Pc, 0, 0); end; tt(3) = toc / reps; %#ok<NASGU>
say(fid, '  %s per call: AFC %.2f us, BSC %.2f us, PID %.2f us (interpreter overhead dominates)\n', ...
    platform_string(), 1e6 * tt);
emb = fullfile(root, 'embedded', 'results_m4.txt');
if exist(emb, 'file')
    txt = fileread(emb);
    say(fid, '  Embedded C (see embedded/README.md):\n%s\n', txt);
end
% instruction counts from embedded/results_m4.txt (QEMU mps2-an386, Cortex-M4)
instr = [18070 9275 940; 9555 4424 713; 811 206 144];   % rows: soft double, soft float, FPU float
cpi = [1.0 1.5];
fclk = 100e6;
ref = qc_paper_ref();
fig = newfig(17, 8);
hold on; box on; grid on;
tms = 1e3 * instr(1, :) / fclk;
bar(1:3, [ref.comp_time_ms; 1e3 * instr(1, :) * mean(cpi) / fclk; 1e3 * instr(3, :) * mean(cpi) / fclk]');
set(gca, 'YScale', 'log', 'XTick', 1:3, 'XTickLabel', {'AFC', 'BSC', 'PID'});
ylabel('time per sample (ms)');
legend({'paper Table III (MK60D)', 'law only, M4 soft-float double (est.)', 'law only, M4F single (est.)'}, 'Location', 'northeast');
title('Control-law cost: the paper''s times are dominated by something other than the law');
qc_save_fig(fig, fullfile(out, 'fig_timing'));
close(fig);
S.timing.matlab_s = tt;
S.timing.instr = instr;
S.timing.est_ms_soft_double = [tms * cpi(1); tms * cpi(2)];

save(fullfile(out, 'phase2_results.mat'), 'S', '-v7');
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

function fig = newfig(w, h)
fig = figure('Visible', 'off', 'Color', 'w', 'Units', 'centimeters', ...
    'Position', [2 2 w h], 'PaperUnits', 'centimeters', 'PaperSize', [w h], ...
    'PaperPosition', [0 0 w h]);
end

function plot_sensing(SB, NS, mlab, ST, out)
cols = {[0 0 0], ST.pid, ST.afc, ST.bsc};
fig = newfig(17, 14);
subplot(2, 2, [1 2]); hold on; box on; grid on;
r = SB{1};
k = r.t >= 4 & r.t <= 8;
plot(r.t(k), r.x(2, k), 'k-', 'LineWidth', 1.5);
for i = 2:4
    plot(SB{i}.t(k), SB{i}.x_meas(2, k), 'Color', cols{i}, 'LineWidth', 0.9);
end
ylabel('body velocity (m/s)'); xlabel('Time (s)');
legend([{'true'}, mlab(2:4)], 'Location', 'southoutside', 'Orientation', 'horizontal');
title('Velocity seen by the controller');
subplot(2, 2, 3); hold on; box on; grid on;
for i = 1:3
    plot(NS.scales, NS.ev(i, :), '-o', 'Color', cols{i + 1}, 'MarkerFaceColor', cols{i + 1}, 'LineWidth', ST.lw);
end
xlabel('noise scale (x nominal)'); ylabel('velocity error RMS (m/s)');
legend({'differencing', 'integration', 'Kalman'}, 'Location', 'northwest');
subplot(2, 2, 4); hold on; box on; grid on;
for i = 1:3
    plot(NS.scales, NS.acc(i, :), '-o', 'Color', cols{i + 1}, 'MarkerFaceColor', cols{i + 1}, 'LineWidth', ST.lw);
end
xlabel('noise scale (x nominal)'); ylabel('body acc. RMS (m/s^2)');
title('Comfort cost of sensor noise');
qc_save_fig(fig, fullfile(out, 'fig_sensing'));
close(fig);
end
