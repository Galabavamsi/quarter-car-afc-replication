function S = run_phase3b()
%RUN_PHASE3B Half-car extension (paper Remark 3): decentralised AFC on two
%   calibrated servo-valve corners, bump and Case-10 roads, against passive,
%   skyhook and the anti-windup AFC. Outputs: results/phase3b/.
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'afc2'));
out = fullfile(root, 'results', 'phase3b');
if ~exist(out, 'dir'), mkdir(out); end
ST = qc_style();
fid = fopen(fullfile(out, 'phase3b_summary.txt'), 'w');
cleaner = onCleanup(@() fclose(fid));
ctrls = {'passive', 'afc', 'afc_aw', 'skyhook'};
names = {'Passive', 'AFC (per corner)', 'AFC-AW (per corner)', 'Skyhook'};
cols = {ST.passive, ST.afc, ST.bsc, ST.pid};
roads = {'bump', 'case10'};
rows = {};
for r = 1:2
    say(fid, '\n== Half car, road: %s (rear wheel delayed by (a+b)/v) ==\n', roads{r});
    for c = 1:4
        R = qc_halfcar(ctrls{c}, roads{r});
        S.(roads{r}).(ctrls{c}) = R;
        m = R.metrics;
        say(fid, '  %-20s heave acc RMS %.3f  pitch acc RMS %.3f rad/s^2  max pitch %.2f deg  peak front %.1f / rear %.1f mm  bound viol %.1f / %.1f mm  sat %.0f%%  uRMS %.2f V\n', ...
            names{c}, m.heave_rms, m.pitch_acc_rms, m.pitch_max_deg, 1e3 * m.peak_front, 1e3 * m.peak_rear, ...
            1e3 * m.viol_front, 1e3 * m.viol_rear, 100 * m.sat, m.u_rms);
        rows(end + 1, :) = {roads{r}, names{c}, m.heave_rms, m.pitch_acc_rms, m.pitch_max_deg, m.peak_front, m.peak_rear, m.viol_front, m.viol_rear, m.sat, m.u_rms}; %#ok<AGROW>
    end
    fig = figure('Visible', 'off', 'Color', 'w', 'Units', 'centimeters', 'Position', [2 2 17 15], ...
        'PaperUnits', 'centimeters', 'PaperSize', [17 15], 'PaperPosition', [0 0 17 15]);
    lbl = {'front corner z_{sf} (mm)', 'rear corner z_{sr} (mm)', 'pitch (deg)'};
    for p = 1:3
        subplot(3, 1, p); hold on; box on; grid on;
        R0 = S.(roads{r}).afc;
        if p < 3
            plot(R0.t, 1e3 * R0.mu1, '--', 'Color', ST.bound); plot(R0.t, -1e3 * R0.mu1, '--', 'Color', ST.bound);
        end
        for c = 1:4
            R = S.(roads{r}).(ctrls{c});
            switch p
                case 1, y = 1e3 * R.zsf;
                case 2, y = 1e3 * R.zsr;
                otherwise, y = R.x(3, :) * 180 / pi;
            end
            plot(R.t, y, 'Color', cols{c}, 'LineWidth', 1.1);
        end
        ylabel(lbl{p});
        if p == 1
            title(sprintf('Half car (I = %g kg m^2, a = %.1f m, b = %.1f m), road: %s', R0.H.I, R0.H.a, R0.H.b, roads{r}));
            if strcmp(roads{r}, 'bump'), ylim([-80 80]); end
        end
        if strcmp(roads{r}, 'case10'), xlim([0 10]); end
    end
    xlabel('Time (s)');
    subplot(3, 1, 3); legend(names, 'Location', 'southeast');
    qc_save_fig(fig, fullfile(out, ['fig_halfcar_', roads{r}]));
    close(fig);
end
qc_write_csv(fullfile(out, 'halfcar.csv'), {'road', 'controller', 'heave_acc_rms', 'pitch_acc_rms', ...
    'pitch_max_deg', 'peak_front_m', 'peak_rear_m', 'viol_front_m', 'viol_rear_m', 'sat_frac', 'u_rms'}, rows);
save(fullfile(out, 'phase3b_results.mat'), 'S', '-v7');
end

function say(fid, fmt, varargin)
fprintf(fmt, varargin{:});
fprintf(fid, fmt, varargin{:});
end
