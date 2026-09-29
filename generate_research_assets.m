function generate_research_assets(includeSimulinkRun)
% Rebuild publication figures and capture the editable Simulink model.
if nargin < 1
    includeSimulinkRun = true;
end
load('quarter_car_replication_results.mat', 'results');
out = fullfile(pwd, 'paper_assets');
if ~exist(out, 'dir')
    mkdir(out);
end

colors.afc = [0.00 0.40 0.55];
colors.bsc = [0.77 0.33 0.12];
colors.pid = [0.36 0.38 0.54];
colors.passive = [0.38 0.42 0.42];
colors.road = [0.18 0.23 0.24];
colors.bound = [0.65 0.12 0.17];
main = results.main;
t = main.afc.t;

fig = new_figure(7.0, 4.4);
tl = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = nexttile(tl);
plot(ax, t, 1000 * main.afc.road, 'Color', colors.road, 'LineWidth', 1.25);
ylabel(ax, 'Road z_r (mm)');
axis_style(ax, t);
ax = nexttile(tl);
hold(ax, 'on');
plot(ax, t, 1000 * main.afc.x(1, :)', 'Color', colors.afc, 'LineWidth', 1.45);
plot(ax, t, 1000 * main.backstepping.x(1, :)', '--', 'Color', colors.bsc, 'LineWidth', 1.2);
plot(ax, t, 1000 * main.pid.x(1, :)', '-.', 'Color', colors.pid, 'LineWidth', 1.3);
plot(ax, t, 1000 * main.passive.x(1, :)', ':', 'Color', colors.passive, 'LineWidth', 1.5);
ylabel(ax, 'Body z_s (mm)');
xlabel(ax, 'Time (s)');
axis_style(ax, t);
legend(ax, {'AFC', 'Backstepping', 'PID', 'Passive'}, ...
    'Location', 'northoutside', 'Orientation', 'horizontal', 'Box', 'off', ...
    'FontSize', 8);
export_pair(fig, out, 'response_comparison');
try
    set(fig, 'Units', 'pixels', 'Position', [80 80 1344 844]);
    drawnow;
    frame = getframe(fig);
    imwrite(frame.cdata, fullfile(out, 'matlab_figure_capture.png'));
catch
    print(fig, fullfile(out, 'matlab_figure_capture.png'), '-dpng', '-r180');
end
close(fig);

fig = new_figure(7.0, 4.15);
tl = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = nexttile(tl);
hold(ax, 'on');
plot(ax, t, 1000 * main.afc.x(1, :)', 'Color', colors.afc, 'LineWidth', 1.35);
plot(ax, t, 1000 * main.afc.mu1, '--', 'Color', colors.bound, 'LineWidth', 1.1);
plot(ax, t, -1000 * main.afc.mu1, '--', 'Color', colors.bound, 'LineWidth', 1.1, ...
    'HandleVisibility', 'off');
ylabel(ax, 'Displacement (mm)');
axis_style(ax, t);
legend(ax, {'AFC z_s', 'Prescribed bounds'}, 'Location', 'northeast', ...
    'Box', 'off', 'FontSize', 8);
ax = nexttile(tl);
hold(ax, 'on');
ratio = abs(main.afc.x(1, :)') ./ main.afc.mu1;
plot(ax, t, ratio, 'Color', colors.afc, 'LineWidth', 1.3);
yline(ax, 1, '--', 'Color', colors.bound, 'LineWidth', 1.15);
ylabel(ax, '|z_s| / \mu_1');
xlabel(ax, 'Time (s)');
axis_style(ax, t);
ylim(ax, [0 max(1.18, 1.05 * max(ratio))]);
export_pair(fig, out, 'ppf_audit');
close(fig);

fig = new_figure(7.0, 4.2);
tl = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = nexttile(tl);
hold(ax, 'on');
plot(ax, t, main.afc.sprung_acceleration, 'Color', colors.afc, 'LineWidth', 1.1);
plot(ax, t, main.passive.sprung_acceleration, ':', 'Color', colors.passive, 'LineWidth', 1.3);
ylabel(ax, 'Body accel. (m/s^2)');
axis_style(ax, t);
legend(ax, {'AFC', 'Passive'}, 'Location', 'northoutside', ...
    'Orientation', 'horizontal', 'Box', 'off', 'FontSize', 8);
ax = nexttile(tl);
hold(ax, 'on');
stairs(ax, t, main.afc.u, 'Color', colors.afc, 'LineWidth', 1.05);
yline(ax, 5, '--', 'Color', colors.bound, 'LineWidth', 0.9);
yline(ax, -5, '--', 'Color', colors.bound, 'LineWidth', 0.9);
ylabel(ax, 'Valve command (V)');
xlabel(ax, 'Time (s)');
axis_style(ax, t);
ylim(ax, [-5.6 5.6]);
export_pair(fig, out, 'acceleration_control');
close(fig);

names = {'AFC', 'BSC', 'PID', 'Passive'};
runs = {main.afc, main.backstepping, main.pid, main.passive};
barColors = [colors.afc; colors.bsc; colors.pid; colors.passive];
metricFields = {'IAE', 'ITAE', 'ITSE', 'acceleration_rms'};
metricLabels = {'IAE (m s)', 'ITAE (m s^2)', 'ITSE (m^2 s^2)', ...
    'RMS acceleration (m/s^2)'};
fig = new_figure(7.0, 4.0);
tl = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for j = 1:4
    ax = nexttile(tl);
    vals = cellfun(@(r) r.metrics.(metricFields{j}), runs);
    bh = bar(ax, vals, 0.65, 'FaceColor', 'flat');
    bh.CData = barColors;
    ax.XTick = 1:4;
    ax.XTickLabel = names;
    ax.XTickLabelRotation = 18;
    ylabel(ax, metricLabels{j});
    ylim(ax, [0 1.18 * max(vals)]);
    set(ax, 'FontName', 'Arial', 'FontSize', 8, 'Box', 'off', ...
        'YGrid', 'on', 'GridAlpha', 0.15, 'TickDir', 'out');
end
export_pair(fig, out, 'metric_comparison');
close(fig);

sweep = results.roadSweep;
peaks = [sweep.peaks];
fields = {'IAE', 'ITAE', 'ITSE'};
labels = {'IAE (m s)', 'ITAE (m s^2)', 'ITSE (m^2 s^2)'};
fig = new_figure(7.0, 2.65);
tl = tiledlayout(fig, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
for j = 1:3
    ax = nexttile(tl);
    vals = arrayfun(@(r) r.metrics.(fields{j}), sweep);
    plot(ax, peaks, vals, '-o', 'Color', colors.afc, 'LineWidth', 1.2, ...
        'MarkerFaceColor', colors.afc, 'MarkerSize', 3.6);
    xlim(ax, [3 10]);
    xticks(ax, 3:10);
    xlabel(ax, 'Road peaks');
    ylabel(ax, labels{j});
    set(ax, 'FontName', 'Arial', 'FontSize', 8, 'Box', 'off', ...
        'XGrid', 'on', 'YGrid', 'on', 'GridAlpha', 0.15, 'TickDir', 'out');
end
export_pair(fig, out, 'road_sweep');
close(fig);

model = 'QuarterCar_AFC_Replication';
load_system(model);
set_param(model, 'ZoomFactor', 'FitSystem');
print(['-s' model], '-dpng', '-r220', fullfile(out, 'simulink_model.png'));
modelImage = imread(fullfile(out, 'simulink_model.png'));
rowSplit = round(0.44 * size(modelImage, 1));
controlRow = trim_white_border(modelImage(1:rowSplit, :, :), 25);
plantRow = trim_white_border(modelImage(rowSplit:end, :, :), 25);
imwrite(controlRow, ...
    fullfile(out, 'simulink_control_row.png'));
imwrite(plantRow, ...
    fullfile(out, 'simulink_plant_row.png'));

if includeSimulinkRun
    simOut = sim(model);
    simTime = simOut.x_states.Time;
    simX = squeeze(simOut.x_states.Data);
    if size(simX, 1) ~= numel(simTime)
        simX = simX.';
    end
    simZ = interp1(simTime, simX(:, 1), t, 'linear');
    delta = simZ - main.afc.x(1, :)';
    comparison.max_abs_body_error_m = max(abs(delta));
    comparison.rms_body_error_m = sqrt(mean(delta.^2));
    comparison.solver = get_param(model, 'Solver');
    comparison.fixed_step_s = str2double(get_param(model, 'FixedStep'));
    comparison.sample_time_s = results.config.sim.sample_time_s;
    save(fullfile(out, 'simulink_crosscheck.mat'), 'comparison');
    fprintf('Simulink/MATLAB max |body displacement difference|: %.6g m\n', ...
        comparison.max_abs_body_error_m);
    fig = new_figure(7.0, 2.65);
    ax = axes(fig);
    hold(ax, 'on');
    plot(ax, t, 1000 * main.afc.x(1, :)', 'Color', colors.afc, 'LineWidth', 1.4);
    plot(ax, t, 1000 * simZ, '--', 'Color', colors.bsc, 'LineWidth', 1.15);
    xlabel(ax, 'Time (s)');
    ylabel(ax, 'Body z_s (mm)');
    axis_style(ax, t);
    legend(ax, {'MATLAB RK4', 'Simulink ode4'}, 'Location', 'northoutside', ...
        'Orientation', 'horizontal', 'Box', 'off', 'FontSize', 8);
    export_pair(fig, out, 'implementation_crosscheck');
    close(fig);
end
close_system(model, 0);
fprintf('Research figures saved to %s\n', out);
end

function fig = new_figure(widthIn, heightIn)
fig = figure('Visible', 'off', 'Color', 'w', ...
    'Units', 'inches', 'Position', [1 1 widthIn heightIn], ...
    'PaperUnits', 'inches', 'PaperPosition', [0 0 widthIn heightIn], ...
    'InvertHardcopy', 'off');
end

function axis_style(ax, t)
xlim(ax, [t(1) t(end)]);
set(ax, 'FontName', 'Arial', 'FontSize', 8.5, 'Box', 'off', ...
    'XGrid', 'on', 'YGrid', 'on', 'GridAlpha', 0.15, 'TickDir', 'out');
end

function export_pair(fig, out, stem)
exportgraphics(fig, fullfile(out, [stem '.pdf']), 'ContentType', 'vector');
exportgraphics(fig, fullfile(out, [stem '.png']), 'Resolution', 300);
end

function cropped = trim_white_border(rgb, padding)
[row, col] = find(any(rgb < 245, 3));
top = max(1, min(row) - padding);
bottom = min(size(rgb, 1), max(row) + padding);
left = max(1, min(col) - padding);
right = min(size(rgb, 2), max(col) + padding);
cropped = rgb(top:bottom, left:right, :);
end
