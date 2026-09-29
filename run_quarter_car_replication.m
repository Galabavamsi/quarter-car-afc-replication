function results = run_quarter_car_replication(cfg, includeRoadSweep)
% Sampled-data replication of the quarter-car AFC paper.
% Defaults are literature-based representative values, not original rig data.
if nargin < 1 || isempty(cfg)
    cfg = quarter_car_replication_config();
end
if nargin < 2
    includeRoadSweep = true;
end
roadCases = table((3:10)', [0.025; repmat(0.043, 7, 1)], ...
    [0.188; 0.204; 0.243; 0.307; 0.361; 0.407; 0.423; 0.505], ...
    'VariableNames', {'Peaks', 'Amplitude_m', 'Frequency_Hz'});

mainRoad = roadCases(end, :);
results.config = cfg;
results.roadCases = roadCases;
results.main.afc = simulate_case(cfg, mainRoad, 'afc');
results.main.backstepping = simulate_case(cfg, mainRoad, 'backstepping');
results.main.pid = simulate_case(cfg, mainRoad, 'pid');
results.main.passive = simulate_case(cfg, mainRoad, 'passive');

if includeRoadSweep
    for i = 1:height(roadCases)
        run = simulate_case(cfg, roadCases(i, :), 'afc');
        results.roadSweep(i).peaks = roadCases.Peaks(i); %#ok<AGROW>
        results.roadSweep(i).amplitude_m = roadCases.Amplitude_m(i);
        results.roadSweep(i).frequency_Hz = roadCases.Frequency_Hz(i);
        results.roadSweep(i).metrics = run.metrics;
    end
else
    results.roadSweep = struct([]);
end

figureFolder = fullfile(pwd, 'replication_figures');
if ~exist(figureFolder, 'dir')
    mkdir(figureFolder);
end
mainFigure = plot_main_case(results.main, cfg);
exportgraphics(mainFigure, fullfile(figureFolder, 'main_case.png'), ...
    'Resolution', 200);
if includeRoadSweep
    sweepFigure = plot_road_sweep(results.roadSweep);
    exportgraphics(sweepFigure, fullfile(figureFolder, 'road_sweep.png'), ...
        'Resolution', 200);
end
save('quarter_car_replication_results.mat', 'results');

fprintf('\nMain case: %d peaks, A = %.3f m, f = %.3f Hz\n', ...
    mainRoad.Peaks, mainRoad.Amplitude_m, mainRoad.Frequency_Hz);
fprintf('%-16s %10s %10s %10s %10s %10s\n', ...
    'Controller', 'IAE', 'ITAE', 'ITSE', 'RMS acc.', 'Peak |u|');
print_metrics('AFC', results.main.afc.metrics);
print_metrics('Backstepping', results.main.backstepping.metrics);
print_metrics('PID', results.main.pid.metrics);
print_metrics('Passive', results.main.passive.metrics);
fprintf('AFC saturation: %.1f%% of samples\n', ...
    100 * results.main.afc.metrics.saturation_fraction);
fprintf('Maximum AFC PPF violation: %.4g m\n', ...
    results.main.afc.metrics.ppf_violation_m);
fprintf('Saved quarter_car_replication_results.mat\n');
end

function run = simulate_case(cfg, roadCase, controllerName)
Ts = cfg.sim.sample_time_s;
N = round(cfg.sim.duration_s / Ts);
substeps = cfg.sim.integration_substeps;
h = Ts / substeps;
t = (0:N)' * Ts;
x = zeros(5, N + 1);
x(:, 1) = cfg.sim.initial_state;
u = zeros(N + 1, 1);
uRaw = zeros(N + 1, 1);
pidIntegral = 0;

for k = 1:N
    [uRaw(k), pidIntegral] = controller_output( ...
        cfg, t(k), x(:, k), roadCase, controllerName, pidIntegral);
    u(k) = min(max(uRaw(k), -cfg.sim.voltage_limit_V), ...
        cfg.sim.voltage_limit_V);

    xk = x(:, k);
    for j = 1:substeps
        tj = t(k) + (j - 1) * h;
        k1 = plant_rhs(cfg, tj, xk, u(k), roadCase, controllerName);
        k2 = plant_rhs(cfg, tj + h / 2, xk + h * k1 / 2, ...
            u(k), roadCase, controllerName);
        k3 = plant_rhs(cfg, tj + h / 2, xk + h * k2 / 2, ...
            u(k), roadCase, controllerName);
        k4 = plant_rhs(cfg, tj + h, xk + h * k3, ...
            u(k), roadCase, controllerName);
        xk = xk + h * (k1 + 2 * k2 + 2 * k3 + k4) / 6;
    end
    x(:, k + 1) = xk;
end
u(end) = u(end - 1);
uRaw(end) = uRaw(end - 1);

road = zeros(size(t));
roadVelocity = zeros(size(t));
sprungAcceleration = zeros(size(t));
for k = 1:numel(t)
    [road(k), roadVelocity(k)] = road_signal(t(k), roadCase);
    dx = plant_rhs(cfg, t(k), x(:, k), u(k), roadCase, controllerName);
    sprungAcceleration(k) = dx(2);
end

run.t = t;
run.x = x;
run.u = u;
run.u_raw = uRaw;
run.road = road;
run.road_velocity = roadVelocity;
run.sprung_acceleration = sprungAcceleration;
run.mu1 = ppf(cfg, t, 1);
run.controller = controllerName;
run.metrics = calculate_metrics(run, cfg);
end

function [uRaw, pidIntegral] = controller_output( ...
    cfg, t, x, roadCase, controllerName, pidIntegral)
switch controllerName
    case 'afc'
        delta = cfg.afc.delta;
        deltaBar = cfg.afc.delta_bar;
        mu = ppf(cfg, t, 1:3);
        zeta1 = bounded_ratio(x(1) / mu(1), delta, deltaBar);
        epsilon1 = inverse_transform(zeta1, delta, deltaBar);
        u1 = -cfg.afc.k(1) * epsilon1;

        zeta2 = bounded_ratio((x(2) - u1) / mu(2), delta, deltaBar);
        epsilon2 = inverse_transform(zeta2, delta, deltaBar);
        u2 = -cfg.afc.k(2) * epsilon2;

        zeta3 = bounded_ratio((x(5) - u2) / mu(3), delta, deltaBar);
        epsilon3 = inverse_transform(zeta3, delta, deltaBar);
        uRaw = -cfg.afc.k(3) * epsilon3;

    case 'backstepping'
        if ~strcmp(cfg.actuator.model, 'equivalent')
            error('Backstepping comparison requires the equivalent actuator model.');
        end
        p = cfg.plant;
        [zr, zrdot] = road_signal(t, roadCase);
        dx = plant_rhs(cfg, t, x, 0, roadCase, controllerName);
        [~, ~, forceSpring, forceDamper] = mechanical_forces(p, x, zr, zrdot);
        alpha1 = -cfg.backstepping.k(1) * x(1);
        alpha1dot = -cfg.backstepping.k(1) * x(2);
        error2 = x(2) - alpha1;
        alpha2 = p.kappa / p.area_m2 * ...
            (-p.ms_kg * cfg.backstepping.k(2) * error2 + ...
            forceDamper + forceSpring + p.ms_kg * alpha1dot);

        alpha1ddot = -cfg.backstepping.k(1) * dx(2);
        forceDamperDot = p.bs_Ns_m * (dx(2) - dx(4));
        suspensionDeflection = x(1) - x(3);
        forceSpringDot = (p.ks_N_m + 3 * p.ksn_N_m3 * ...
            suspensionDeflection^2) * (x(2) - x(4));
        alpha2dot = p.kappa / p.area_m2 * ...
            (-p.ms_kg * cfg.backstepping.k(2) * ...
            (dx(2) - alpha1dot) + forceDamperDot + forceSpringDot + ...
            p.ms_kg * alpha1ddot);
        error3 = x(5) - alpha2;
        [actuatorDrift, actuatorInputGain] = actuator_terms(cfg, x, 0);
        uRaw = (-cfg.backstepping.k(3) * error3 + alpha2dot - ...
            actuatorDrift) / actuatorInputGain;

    case 'pid'
        error = -x(1);
        candidateIntegral = pidIntegral + cfg.sim.sample_time_s * error;
        uCandidate = cfg.pid.kp * error + cfg.pid.ki * candidateIntegral ...
            - cfg.pid.kd * x(2);
        limit = cfg.sim.voltage_limit_V;
        if abs(uCandidate) <= limit || sign(uCandidate) ~= sign(error)
            pidIntegral = candidateIntegral;
        end
        uRaw = cfg.pid.kp * error + cfg.pid.ki * pidIntegral ...
            - cfg.pid.kd * x(2);

    case 'passive'
        uRaw = 0;

    otherwise
        error('Unknown controller: %s', controllerName);
end
end

function dx = plant_rhs(cfg, t, x, u, roadCase, controllerName)
p = cfg.plant;
[zr, zrdot] = road_signal(t, roadCase);
[tireForce, tireDamperForce, springForce, damperForce] = ...
    mechanical_forces(p, x, zr, zrdot);

if strcmp(controllerName, 'passive')
    actuatorForce = 0;
    dx5 = 0;
else
    actuatorForce = p.area_m2 / p.kappa * x(5);
    [actuatorDrift, actuatorInputGain] = actuator_terms(cfg, x, u);
    dx5 = actuatorDrift + actuatorInputGain * u;
    if strcmp(cfg.actuator.model, 'equivalent')
        pressureStateLimit = p.kappa * cfg.actuator.pressure_limit_Pa;
        if (x(5) >= pressureStateLimit && dx5 > 0) || ...
                (x(5) <= -pressureStateLimit && dx5 < 0)
            dx5 = 0;
        end
    end
end

dx = zeros(5, 1);
dx(1) = x(2);
dx(2) = (-damperForce - springForce + actuatorForce) / p.ms_kg;
dx(3) = x(4);
dx(4) = (damperForce + springForce - tireForce - ...
    tireDamperForce - actuatorForce) / p.mu_kg;
dx(5) = dx5;
end

function [drift, inputGain] = actuator_terms(cfg, x, u)
a = cfg.actuator;
p = cfg.plant;
switch a.model
    case 'equivalent'
        drift = -a.beta_per_s * x(5) - a.cRel * (x(2) - x(4));
        inputGain = a.bu_per_s_V;
    case 'paper-valve'
        constants = [a.a, a.gamma, a.supply_pressure_Pa, ...
            a.fluid_density_kg_m3];
        if any(~isfinite(constants)) || any(constants <= 0)
            error('Enter measured values for the paper-valve actuator constants.');
        end
        pressure = x(5) / p.kappa;
        pressureDrop = a.supply_pressure_Pa - sign(u) * pressure;
        valveFlowFactor = sign(pressureDrop) * ...
            sqrt(abs(pressureDrop) / a.fluid_density_kg_m3);
        drift = -a.beta_per_s * x(5) - p.kappa * a.a * ...
            p.area_m2 * (x(2) - x(4));
        inputGain = p.kappa * a.gamma * valveFlowFactor;
    otherwise
        error('Unknown actuator model: %s', a.model);
end
end

function [tireForce, tireDamperForce, springForce, damperForce] = ...
    mechanical_forces(p, x, zr, zrdot)
suspensionDeflection = x(1) - x(3);
suspensionVelocity = x(2) - x(4);
springForce = p.ks_N_m * suspensionDeflection + ...
    p.ksn_N_m3 * suspensionDeflection^3;
damperForce = p.bs_Ns_m * suspensionVelocity;
tireForce = p.kt_N_m * (x(3) - zr);
tireDamperForce = p.bt_Ns_m * (x(4) - zrdot);
end

function [zr, zrdot] = road_signal(t, roadCase)
omega = 2 * pi * roadCase.Frequency_Hz;
zr = roadCase.Amplitude_m * sin(omega * t);
zrdot = roadCase.Amplitude_m * omega * cos(omega * t);
end

function mu = ppf(cfg, t, index)
mu = (cfg.afc.mu0(index) - cfg.afc.mu_inf(index)) .* ...
    exp(-cfg.afc.alpha(index) .* t) + cfg.afc.mu_inf(index);
end

function zeta = bounded_ratio(zeta, delta, deltaBar)
guard = 1e-8 * max([1, delta, deltaBar]);
zeta = min(max(zeta, -delta + guard), deltaBar - guard);
end

function epsilon = inverse_transform(zeta, delta, deltaBar)
epsilon = 0.5 * log((delta + zeta) ./ (deltaBar - zeta));
end

function metrics = calculate_metrics(run, cfg)
t = run.t;
zs = run.x(1, :)';
acc = run.sprung_acceleration;
metrics.IAE = trapz(t, abs(zs));
metrics.ITAE = trapz(t, t .* abs(zs));
metrics.ITSE = trapz(t, t .* zs.^2);
metrics.acceleration_rms = sqrt(mean(acc.^2));
metrics.acceleration_peak = max(abs(acc));
metrics.control_rms = sqrt(mean(run.u.^2));
metrics.control_peak = max(abs(run.u));
metrics.saturation_fraction = mean(abs(run.u_raw) > cfg.sim.voltage_limit_V);
metrics.ppf_violation_m = max([0; zs - run.mu1; -zs - run.mu1]);
end

function fig = plot_main_case(main, cfg)
afc = main.afc;
fig = figure('Name', 'Quarter-car AFC replication', 'Color', 'w');
tiledlayout(3, 1, 'TileSpacing', 'compact');

nexttile;
plot(afc.t, afc.road, 'Color', [0.35 0.35 0.35], 'LineWidth', 1);
grid on;
ylabel('Road z_r (m)');
title('Road input and sprung-mass response');

nexttile;
hold on;
plot(afc.t, afc.mu1, 'k:', 'LineWidth', 1.2);
plot(afc.t, -afc.mu1, 'k:', 'LineWidth', 1.2, ...
    'HandleVisibility', 'off');
plot(afc.t, afc.x(1, :), 'LineWidth', 1.1);
plot(main.backstepping.t, main.backstepping.x(1, :), '--', ...
    'LineWidth', 0.9);
plot(main.pid.t, main.pid.x(1, :), '-.', 'LineWidth', 0.9);
plot(main.passive.t, main.passive.x(1, :), 'Color', [0.55 0.55 0.55], ...
    'LineStyle', '-', 'LineWidth', 0.8);
grid on;
ylabel('Sprung z_s (m)');
legend('PPF upper/lower', 'AFC', 'Backstepping', 'PID', 'Passive', ...
    'Location', 'best');

nexttile;
yyaxis left;
plot(afc.t, afc.sprung_acceleration, 'LineWidth', 0.9);
ylabel('Body acceleration (m/s^2)');
yyaxis right;
plot(afc.t, afc.u, 'LineWidth', 0.9);
ylabel('Valve voltage (V)');
ylim([-cfg.sim.voltage_limit_V, cfg.sim.voltage_limit_V]);
xlabel('Time (s)');
grid on;
end

function fig = plot_road_sweep(roadSweep)
peaks = [roadSweep.peaks];
iae = arrayfun(@(r) r.metrics.IAE, roadSweep);
itae = arrayfun(@(r) r.metrics.ITAE, roadSweep);
itse = arrayfun(@(r) r.metrics.ITSE, roadSweep);
fig = figure('Name', 'AFC road-condition sweep', 'Color', 'w');
tiledlayout(1, 3, 'TileSpacing', 'compact');
nexttile; plot(peaks, iae, '-o', 'LineWidth', 1.1); grid on;
xlabel('Road case (peaks)'); ylabel('IAE (m s)');
nexttile; plot(peaks, itae, '-o', 'LineWidth', 1.1); grid on;
xlabel('Road case (peaks)'); ylabel('ITAE (m s^2)');
nexttile; plot(peaks, itse, '-o', 'LineWidth', 1.1); grid on;
xlabel('Road case (peaks)'); ylabel('ITSE (m^2 s^2)');
end

function print_metrics(name, metrics)
fprintf('%-16s %10.4g %10.4g %10.4g %10.4g %10.4g\n', ...
    name, metrics.IAE, metrics.ITAE, metrics.ITSE, ...
    metrics.acceleration_rms, metrics.control_peak);
end
