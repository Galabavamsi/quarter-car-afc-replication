function S = run_phase2b()
%RUN_PHASE2B Cross-check the hand-written plant against Simscape Multibody.
%   Builds QuarterCar_Multibody.slx (three prismatic joints, bricks with the
%   model masses, servo-valve dynamics in Simulink, 50 Hz AFC), simulates
%   paper Case 10 and compares body displacement with qc_simulate.
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'afc2'));
out = fullfile(root, 'results', 'phase2b');
if ~exist(out, 'dir'), mkdir(out); end
model = build_quarter_car_multibody(true);
so = sim(model);
xs = so.mb_x; us = so.mb_u;
D = squeeze(xs.Data);
if size(D, 1) == 6 && size(D, 2) ~= 6, D = D'; end
P = qc_params('alleyne-cal');
r = qc_simulate(P, qc_road('case', 10), qc_ctrl('afc', 'A'));
zsMB = interp1(xs.Time, D(:, 1), r.t, 'linear');
dz = max(abs(zsMB - r.x(1, :)'));
iaeMB = trapz(r.t, abs(zsMB));
fprintf('Simscape Multibody vs qc_simulate (Case 10, AFC A): max|dzs| = %.3g m; IAE %.4f vs %.4f\n', dz, iaeMB, r.metrics.IAE);
fid = fopen(fullfile(out, 'phase2b_summary.txt'), 'w');
fprintf(fid, 'Simscape Multibody vs qc_simulate (Case 10, AFC A): max|dzs| = %.3g m; IAE %.4f vs %.4f\n', dz, iaeMB, r.metrics.IAE);
fclose(fid);
fig = figure('Visible', 'off', 'Color', 'w');
plot(r.t, 1e3 * r.x(1, :), 'LineWidth', 1.4); hold on;
plot(r.t, 1e3 * zsMB, '--', 'LineWidth', 1.2);
legend({'qc\_simulate (RK4, equations)', 'Simscape Multibody (ode15s)'}); grid on;
xlabel('Time (s)'); ylabel('z_s (mm)'); title(sprintf('Independent physics cross-check, max difference %.2g mm', 1e3 * dz));
qc_save_fig(fig, fullfile(out, 'fig_multibody_crosscheck'));
close(fig);
S = struct('dz', dz, 'iae_mb', iaeMB, 'iae_eq', r.metrics.IAE);
end
