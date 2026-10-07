function modelName = build_quarter_car_simulink_v2(overwrite, plantName, variant, sensing)
%BUILD_QUARTER_CAR_SIMULINK_V2 Editable Simulink model of the v2 study.
%
%   build_quarter_car_simulink_v2(true)                       calibrated plant, AFC Case A, ideal sensing
%   build_quarter_car_simulink_v2(true, 'alleyne-cal', 'A', 'accint')
%
%   Diagram (left to right):
%     Digital Clock (50 Hz) --+
%     ZOH(x), ZOH(a_s) -------+--> [Digital controller: sensors + AFC]  --> Saturation (+/-5 V) --+
%     From Workspace (noise) -+                                                                   |
%     Clock --> [Road] --> zr, zrdot -----------------------------------------------+             |
%                                                                                   v             v
%                                                  [Quarter car + servo valve: dx = f(x,u,zr,zrdot)] --> 1/s --> x
%                                                  [Accelerometer: a_s = g(x)]  (no u, so no algebraic loop)
%
%   Constants are embedded from qc_params/qc_ctrl when the model is built,
%   exactly like v1. Rebuild after changing parameters. The noise input is
%   the workspace variable qc_noise = [t, n_z, n_a, n_p] (unit-variance
%   columns); run_phase2 supplies it so Simulink and qc_simulate see the
%   SAME noise and can be compared sample by sample.
if nargin < 1, overwrite = false; end
if nargin < 2 || isempty(plantName), plantName = 'alleyne-cal'; end
if nargin < 3 || isempty(variant), variant = 'A'; end
if nargin < 4 || isempty(sensing), sensing = 'ideal'; end
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'afc2'));
P = qc_params(plantName);
C = qc_ctrl('afc', variant);
if ~strcmp(P.act.model, 'valve')
    error('The v2 Simulink model implements the servo-valve actuator; use the v1 model for the equivalent actuator.');
end
modeId = find(strcmp(sensing, {'ideal', 'accint'}));
if isempty(modeId)
    error('sensing must be ''ideal'' or ''accint'' in the Simulink model.');
end

modelName = 'QuarterCar_AFC_v2';
modelFile = fullfile(root, [modelName, '.slx']);
if exist(modelFile, 'file')
    if ~overwrite
        error('File exists: %s. Pass true to rebuild it.', modelFile);
    end
    if bdIsLoaded(modelName)
        if strcmp(get_param(modelName, 'Dirty'), 'on')
            error('The loaded model has unsaved edits; save or close it first.');
        end
        close_system(modelName, 0);
    end
    delete(modelFile);
end
Ts = P.sim.Ts;
new_system(modelName);
cleanupModel = onCleanup(@() close_system(modelName, 0));
set_param(modelName, 'StopTime', num2str(P.sim.T), 'Solver', 'ode4', ...
    'FixedStep', num2str(P.sim.h), 'SaveTime', 'on', 'TimeSaveName', 'tout');
m = modelName;

% ---- sources -------------------------------------------------------------
add_block('simulink/Sources/Clock', [m '/Clock'], 'Position', [30 420 60 450]);
add_block('simulink/User-Defined Functions/MATLAB Function', [m '/Road'], 'Position', [110 405 220 465]);
add_block('simulink/Sources/Digital Clock', [m '/Digital Clock'], 'SampleTime', num2str(Ts), 'Position', [30 60 80 90]);
add_block('simulink/Sources/From Workspace', [m '/Sensor noise'], 'VariableName', 'qc_noise', ...
    'SampleTime', num2str(Ts), 'Interpolate', 'off', 'OutputAfterFinalValue', 'Holding final value', ...
    'Position', [20 220 110 250]);
% ---- digital controller --------------------------------------------------
add_block('simulink/Discrete/Zero-Order Hold', [m '/Sample x'], 'SampleTime', num2str(Ts), 'Position', [150 105 185 135]);
add_block('simulink/Discrete/Zero-Order Hold', [m '/Sample a_s'], 'SampleTime', num2str(Ts), 'Position', [150 160 185 190]);
add_block('simulink/User-Defined Functions/MATLAB Function', [m '/Digital controller (sensors + AFC)'], 'Position', [250 60 470 260]);
add_block('simulink/Discontinuities/Saturation', [m '/Valve rail +-5 V'], 'UpperLimit', num2str(P.sim.Vmax), ...
    'LowerLimit', num2str(-P.sim.Vmax), 'Position', [530 75 570 105]);
% ---- plant ------------------------------------------------------------------
add_block('simulink/User-Defined Functions/MATLAB Function', [m '/Quarter car + servo valve'], 'Position', [620 330 820 470]);
add_block('simulink/Continuous/Integrator', [m '/States'], 'InitialCondition', 'zeros(6,1)', 'Position', [880 385 920 415]);
add_block('simulink/User-Defined Functions/MATLAB Function', [m '/Accelerometer (true)'], 'Position', [880 160 1010 210]);
add_block('simulink/Signal Routing/Goto', [m '/x tag'], 'GotoTag', 'qc_x', 'TagVisibility', 'global', 'Position', [970 390 1030 410]);
add_block('simulink/Signal Routing/From', [m '/x for controller'], 'GotoTag', 'qc_x', 'Position', [60 110 120 130]);
add_block('simulink/Signal Routing/From', [m '/x for plant'], 'GotoTag', 'qc_x', 'Position', [520 340 580 360]);
add_block('simulink/Signal Routing/From', [m '/x for accelerometer'], 'GotoTag', 'qc_x', 'Position', [800 175 860 195]);
add_block('simulink/Signal Routing/Goto', [m '/a tag'], 'GotoTag', 'qc_a', 'TagVisibility', 'global', 'Position', [1060 165 1120 185]);
add_block('simulink/Signal Routing/From', [m '/a for controller'], 'GotoTag', 'qc_a', 'Position', [60 165 120 185]);
% ---- logs -------------------------------------------------------------------
sinks = {'u log', 'qc_u', [620 70 700 90]; 'u_raw log', 'qc_u_raw', [530 130 610 150]; ...
    'zeta log', 'qc_zeta', [530 170 610 190]; 'x_meas log', 'qc_xm', [530 220 610 240]; ...
    'x log', 'qc_xlog', [970 440 1050 460]; 'a log', 'qc_alog', [1060 205 1140 225]; ...
    'road log', 'qc_road', [270 480 350 500]};
for i = 1:size(sinks, 1)
    add_block('simulink/Sinks/To Workspace', [m '/' sinks{i, 1}], 'VariableName', sinks{i, 2}, ...
        'SaveFormat', 'Timeseries', 'Position', sinks{i, 3});
end

% ---- code -------------------------------------------------------------------
setScript([m '/Road'], road_code(P, 10));
setScript([m '/Digital controller (sensors + AFC)'], controller_code(P, C, modeId));
setScript([m '/Quarter car + servo valve'], plant_code(P));
setScript([m '/Accelerometer (true)'], accel_code(P));

% ---- wiring -----------------------------------------------------------------
L = @(a, b) add_line(m, a, b, 'autorouting', 'on');
L('Clock/1', 'Road/1');
L('Digital Clock/1', 'Digital controller (sensors + AFC)/1');
L('x for controller/1', 'Sample x/1');
L('Sample x/1', 'Digital controller (sensors + AFC)/2');
L('a for controller/1', 'Sample a_s/1');
L('Sample a_s/1', 'Digital controller (sensors + AFC)/3');
L('Sensor noise/1', 'Digital controller (sensors + AFC)/4');
L('Digital controller (sensors + AFC)/1', 'Valve rail +-5 V/1');
L('Digital controller (sensors + AFC)/1', 'u_raw log/1');
L('Digital controller (sensors + AFC)/2', 'zeta log/1');
L('Digital controller (sensors + AFC)/3', 'x_meas log/1');
L('Valve rail +-5 V/1', 'u log/1');
L('x for plant/1', 'Quarter car + servo valve/1');
L('Valve rail +-5 V/1', 'Quarter car + servo valve/2');
L('Road/1', 'Quarter car + servo valve/3');
L('Road/2', 'Quarter car + servo valve/4');
L('Road/1', 'road log/1');
L('Quarter car + servo valve/1', 'States/1');
L('States/1', 'x tag/1');
L('States/1', 'x log/1');
L('x for accelerometer/1', 'Accelerometer (true)/1');
L('Accelerometer (true)/1', 'a tag/1');
L('Accelerometer (true)/1', 'a log/1');

noteText = sprintf(['v2 model: %s plant, AFC Case %s, sensing = %s. ' ...
    'Generated by build_quarter_car_simulink_v2.m; rebuild after editing parameters.'], plantName, variant, sensing);
try
    note = Simulink.Annotation(m, noteText);
    note.Position = [250 540 750 560];
catch
    % annotations are cosmetic; ignore if this release uses another API
end
assignin('base', 'qc_noise', [0 0 0 0; P.sim.T 0 0 0]);   % default: no noise
set_param(m, 'SimulationCommand', 'update');
save_system(m, modelFile);
clear cleanupModel;
if usejava('desktop')
    open_system(m);   % show the diagram only in an interactive desktop session
end
end

function setScript(blockPath, code)
ch = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', blockPath);
ch.Script = code;
end

function code = road_code(P, peaks) %#ok<INUSL>
tab = [3 0.025 0.188; 4 0.043 0.204; 5 0.043 0.243; 6 0.043 0.307; ...
       7 0.043 0.361; 8 0.043 0.407; 9 0.043 0.423; 10 0.043 0.505];
r = tab(tab(:, 1) == peaks, :);
code = sprintf(['function [zr, zrd] = fcn(t)\n' ...
    '%% Paper road case %d peaks: %.3f m, %.3f Hz\n' ...
    'A = %.16g; w = %.16g;\n' ...
    'zr = A*sin(w*t);\nzrd = A*w*cos(w*t);\nend\n'], peaks, r(2), r(3), r(2), 2 * pi * r(3));
end

function code = plant_code(P)
m = P.mech; a = P.act;
code = sprintf([ ...
    'function dx = fcn(x, u, zr, zrd)\n' ...
    '%% Quarter car (Na et al. eq. 7) + servo-valve flow law (eqs. 2-6) + spool lag\n' ...
    'ms=%.16g; mu=%.16g; ks=%.16g; ksn=%.16g; bs=%.16g; kt=%.16g; bt=%.16g;\n' ...
    'A=%.16g; alpha=%.16g; beta=%.16g; gamma=%.16g; Ps=%.16g; kappa=%.16g; Kv=%.16g; tau=%.16g;\n' ...
    'd=x(1)-x(3); v=x(2)-x(4);\n' ...
    'Fs=ks*d+ksn*d^3; Fd=bs*v; Ft=kt*(x(3)-zr); Fb=bt*(x(4)-zrd);\n' ...
    'PL=x(5)/kappa; F=A*PL;\n' ...
    'dx=zeros(6,1);\n' ...
    'if tau>0\n  xv=x(6); dx(6)=(Kv*u-x(6))/tau;\nelse\n  xv=Kv*u;\nend\n' ...
    'dP=Ps-sign(xv)*PL;\n' ...
    'dx(5)=kappa*(-beta*PL-alpha*A*v+gamma*xv*sqrt(max(dP,0)));\n' ...
    'dx(1)=x(2); dx(2)=(-Fd-Fs+F)/ms; dx(3)=x(4); dx(4)=(Fd+Fs-Ft-Fb-F)/mu;\n' ...
    'end\n'], m.ms, m.mu, m.ks, m.ksn, m.bs, m.kt, m.bt, a.A, a.alpha, a.beta, a.gamma, a.Ps, a.kappa, a.Kv, a.tau);
end

function code = accel_code(P)
m = P.mech; a = P.act;
code = sprintf([ ...
    'function acc = fcn(x)\n' ...
    '%% True body acceleration; depends on states only (no algebraic loop)\n' ...
    'd=x(1)-x(3); v=x(2)-x(4);\n' ...
    'acc=(-%.16g*v-(%.16g*d+%.16g*d^3)+%.16g*x(5)/%.16g)/%.16g;\n' ...
    'end\n'], m.bs, m.ks, m.ksn, a.A, a.kappa, m.ms);
end

function code = controller_code(P, C, modeId)
lam = exp(-P.sim.Ts / 2);
code = sprintf([ ...
    'function [u, zeta, xm] = fcn(t, x, acc, noise)\n' ...
    '%% Runs at 50 Hz (sampled inputs). Sensors (mode %d: 1 ideal, 2 accint) + AFC eqs. (9)-(20)\n' ...
    'persistent vInt\n' ...
    'if isempty(vInt), vInt = 0; end\n' ...
    'mode = %d; Ts = %.16g; lam = %.16g;\n' ...
    'sz = %.16g; sa = %.16g; sp = %.16g; kappa = %.16g;\n' ...
    'xm = x;\n' ...
    'if mode == 2\n' ...
    '  xm(1) = x(1) + sz*noise(1);\n' ...
    '  vInt = lam*vInt + Ts*(acc + sa*noise(2));\n' ...
    '  xm(2) = vInt;\n' ...
    '  xm(5) = x(5) + kappa*sp*noise(3);\n' ...
    'end\n' ...
    'delta = %.16g; dbar = %.16g; g = %.16g;\n' ...
    'mu0 = [%.16g; %.16g; %.16g]; muinf = [%.16g; %.16g; %.16g];\n' ...
    'al = [%.16g; %.16g; %.16g]; k = [%.16g; %.16g; %.16g];\n' ...
    'mu = (mu0 - muinf).*exp(-al*t) + muinf;\n' ...
    'src = [xm(1); xm(2); xm(5)];\n' ...
    'zeta = zeros(3,1); prev = 0; v = 0;\n' ...
    'for i = 1:3\n' ...
    '  zeta(i) = (src(i) - prev)/mu(i);\n' ...
    '  z = min(max(zeta(i), -delta + g), dbar - g);\n' ...
    '  v = -k(i)*0.5*log((delta + z)/(dbar - z));\n' ...
    '  prev = v;\n' ...
    'end\n' ...
    'u = v;\n' ...
    'end\n'], modeId, modeId, P.sim.Ts, lam, 0.5e-3, 0.05, 5e4, P.act.kappa, ...
    C.delta, C.dbar, C.guard, C.mu0, C.muinf, C.alpha, C.k);
end
