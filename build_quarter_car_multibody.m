function modelName = build_quarter_car_multibody(overwrite)
%BUILD_QUARTER_CAR_MULTIBODY Independent Simscape Multibody plant + AFC.
%
%   build_quarter_car_multibody(true)
%
%   The mechanics are NOT written as equations here: Simscape Multibody
%   assembles them from three prismatic joints in series along world Z,
%       World --(road, motion input)--> road platform
%             --(tyre: spring kt)-----> wheel carrier (m_u)
%             --(suspension: spring ks, damper bs, force input F)--> body (m_s)
%   with gravity switched off (the study works about static equilibrium).
%   The servo-valve pressure dynamics and the 50 Hz AFC run in Simulink
%   blocks and push the actuator force into the suspension joint.
%   Comparing its trajectories with qc_simulate (run_phase2b) is a
%   cross-check of the hand-written equations by a different physics engine.
%   Open Mechanics Explorer after a run to see the rig move.
%
%   Requires Simulink + Simscape + Simscape Multibody. If a block parameter
%   name differs in your release, the builder stops and prints the
%   available names: paste that message back so it can be adjusted.
if nargin < 1, overwrite = false; end
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'afc2'));
P = qc_params('alleyne-cal');
C = qc_ctrl('afc', 'A');
m = P.mech; a = P.act;
modelName = 'QuarterCar_Multibody';
modelFile = fullfile(root, [modelName, '.slx']);
if exist(modelFile, 'file')
    if ~overwrite, error('File exists: %s. Pass true to rebuild.', modelFile); end
    if bdIsLoaded(modelName), close_system(modelName, 0); end
    delete(modelFile);
end
new_system(modelName);
cleanupModel = onCleanup(@() close_system(modelName, 0));
mdl = modelName;
set_param(mdl, 'StopTime', num2str(P.sim.T), 'Solver', 'ode15s', 'MaxStep', '1e-3', 'RelTol', '1e-6');

% ---- multibody skeleton -------------------------------------------------
add_block('nesl_utility/Solver Configuration', [mdl '/Solver Configuration'], 'Position', [40 520 100 560]);
add_block('sm_lib/Frames and Transforms/World Frame', [mdl '/World'], 'Position', [40 420 80 460]);
add_block('sm_lib/Utilities/Mechanism Configuration', [mdl '/Mechanism Configuration'], 'Position', [40 600 100 640]);
setp([mdl '/Mechanism Configuration'], '^UniformGravity$', 'None');   % study works about static equilibrium

J = {'Road joint', [160 400 220 470]; 'Tyre joint', [380 400 440 470]; 'Suspension joint', [600 400 660 470]};
for i = 1:3
    add_block('sm_lib/Joints/Prismatic Joint', [mdl '/' J{i, 1}], 'Position', J{i, 2});
end
% road: motion provided by input, force computed
setp([mdl '/Road joint'], 'MotionActuationMode$', 'InputMotion');
setp([mdl '/Road joint'], 'TorqueActuationMode$|ForceActuationMode$', 'ComputedTorque');   % R2026a: force mode is named TorqueActuationMode
% initial state = qc_simulate's x0 = 0: wheel and body at rest while the
% road platform already moves at zr'(0) = A*w, so the tyre joint starts at
% relative velocity -A*w and the suspension joint at 0 (both at q = 0)
Aroad = 0.043; wroad = 2 * pi * 0.505;
for jn = {'Tyre joint', 'Suspension joint'}
    setp([mdl '/' jn{1}], '^PositionTargetSpecify$', 'on');
    setp([mdl '/' jn{1}], '^PositionTargetValue$', '0');
    setp([mdl '/' jn{1}], '^VelocityTargetSpecify$', 'on');
end
setp([mdl '/Tyre joint'], '^VelocityTargetValue$', num2str(-Aroad * wroad, 16));
setp([mdl '/Suspension joint'], '^VelocityTargetValue$', '0');
% tyre: spring only, sense position + velocity
setp([mdl '/Tyre joint'], 'SpringStiffness$', num2str(m.kt, 16));
setp([mdl '/Tyre joint'], 'DampingCoefficient$', num2str(m.bt, 16));
setp([mdl '/Tyre joint'], 'SensePosition$', 'on');
setp([mdl '/Tyre joint'], 'SenseVelocity$', 'on');
% suspension: spring + damper + force input, sense position + velocity
setp([mdl '/Suspension joint'], 'SpringStiffness$', num2str(m.ks, 16));
setp([mdl '/Suspension joint'], 'DampingCoefficient$', num2str(m.bs, 16));
setp([mdl '/Suspension joint'], 'TorqueActuationMode$|ForceActuationMode$', 'InputTorque');
setp([mdl '/Suspension joint'], 'SensePosition$', 'on');
setp([mdl '/Suspension joint'], 'SenseVelocity$', 'on');
setp([mdl '/Road joint'], 'SensePosition$', 'on');
setp([mdl '/Road joint'], 'SenseVelocity$', 'on');

% bodies: bricks whose density gives exactly the model masses
B = {'Road platform', [0.42 0.32 0.025], 1, [0.45 0.45 0.45], [270 300 320 340]; ...
     'Wheel carrier', [0.82 0.11 0.06], m.mu, [0.30 0.45 0.60], [490 300 540 340]; ...
     'Body', [0.86 0.42 0.13], m.ms, [0.00 0.40 0.50], [710 300 760 340]};
for i = 1:3
    blk = [mdl '/' B{i, 1}];
    add_block('sm_lib/Body Elements/Brick Solid', blk, 'Position', B{i, 5});
    dims = B{i, 2};
    setp(blk, 'BrickDimensions$|^Dimensions$', mat2str(dims));
    setp(blk, '^Density$', num2str(B{i, 3} / prod(dims), 16));
    setp(blk, 'GraphicDiffuseColor$', mat2str(B{i, 4}), true);
end

% ---- Simulink side: road, actuator dynamics, controller ------------------
add_block('simulink/Sources/Clock', [mdl '/Clock'], 'Position', [40 200 70 230]);
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/Road'], 'Position', [100 190 180 240]);
add_block('nesl_utility/Simulink-PS Converter', [mdl '/zr to PS'], 'Position', [120 300 140 320]);
% road motion needs zr, zr' and zr''; supply them exactly (no input filter)
setp([mdl '/zr to PS'], 'FilteringAndDerivatives$', 'provide');
setp([mdl '/zr to PS'], 'UdotUserProvided$', '2');
add_block('nesl_utility/PS-Simulink Converter', [mdl '/q_road'], 'Position', [250 520 270 540]);
add_block('nesl_utility/PS-Simulink Converter', [mdl '/v_road'], 'Position', [250 560 270 580]);
add_block('nesl_utility/PS-Simulink Converter', [mdl '/q_tyre'], 'Position', [470 520 490 540]);
add_block('nesl_utility/PS-Simulink Converter', [mdl '/v_tyre'], 'Position', [470 560 490 580]);
add_block('nesl_utility/PS-Simulink Converter', [mdl '/q_susp'], 'Position', [690 520 710 540]);
add_block('nesl_utility/PS-Simulink Converter', [mdl '/v_susp'], 'Position', [690 560 710 580]);
add_block('nesl_utility/Simulink-PS Converter', [mdl '/F to PS'], 'Position', [560 300 580 320]);
setp([mdl '/zr to PS'], '^Unit$', 'm', true);
setp([mdl '/F to PS'], '^Unit$', 'N', true);
for nm = {'q_road', 'q_tyre', 'q_susp'}, setp([mdl '/' nm{1}], '^Unit$', 'm', true); end
for nm = {'v_road', 'v_tyre', 'v_susp'}, setp([mdl '/' nm{1}], '^Unit$', 'm/s', true); end
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/States from joints'], 'Position', [800 500 920 600]);
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/Servo valve + cylinder'], 'Position', [800 160 960 260]);
add_block('simulink/Continuous/Integrator', [mdl '/Valve states'], 'InitialCondition', '[0;0]', 'Position', [1000 190 1040 230]);
add_block('simulink/Sources/Digital Clock', [mdl '/Digital Clock'], 'SampleTime', num2str(P.sim.Ts), 'Position', [1000 40 1050 70]);
add_block('simulink/Discrete/Zero-Order Hold', [mdl '/Sample x'], 'SampleTime', num2str(P.sim.Ts), 'Position', [1000 90 1035 120]);
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/AFC (50 Hz)'], 'Position', [1080 40 1200 130]);
add_block('simulink/Discontinuities/Saturation', [mdl '/Valve rail'], 'UpperLimit', '5', 'LowerLimit', '-5', 'Position', [1240 70 1270 100]);
add_block('simulink/Sinks/To Workspace', [mdl '/x log'], 'VariableName', 'mb_x', 'SaveFormat', 'Timeseries', 'Position', [1000 560 1060 580]);
add_block('simulink/Sinks/To Workspace', [mdl '/u log'], 'VariableName', 'mb_u', 'SaveFormat', 'Timeseries', 'Position', [1300 70 1360 90]);

setScript([mdl '/Road'], sprintf(['function [zr, zrd, zrdd] = fcn(t)\n%% paper road case 10: 0.043 m, 0.505 Hz\n' ...
    'A = %.16g; w = %.16g;\nzr = A*sin(w*t); zrd = A*w*cos(w*t); zrdd = -A*w^2*sin(w*t);\nend\n'], Aroad, wroad));
setScript([mdl '/States from joints'], sprintf([ ...
    'function x = fcn(qr, vr, qt, vt, qs, vs, xa)\n' ...
    '%% series prismatic joints along Z: z_u = q_road + q_tyre, z_s = z_u + q_susp\n' ...
    'x = zeros(6,1);\nx(3) = qr + qt; x(4) = vr + vt;\nx(1) = x(3) + qs; x(2) = x(4) + vs;\nx(5) = xa(1); x(6) = xa(2);\nend\n']));
setScript([mdl '/Servo valve + cylinder'], sprintf([ ...
    'function [dxa, F] = fcn(xa, u, vrel)\n' ...
    '%% Na et al. eqs. (2)-(6) with spool lag; xa = [x5; x_v], x5 = kappa*P_L\n' ...
    'A=%.16g; alpha=%.16g; beta=%.16g; gamma=%.16g; Ps=%.16g; kappa=%.16g; Kv=%.16g; tau=%.16g;\n' ...
    'PL = xa(1)/kappa; xv = xa(2);\n' ...
    'dPL = -beta*PL - alpha*A*vrel + gamma*xv*sqrt(max(Ps - sign(xv)*PL, 0));\n' ...
    'dxa = [kappa*dPL; (Kv*u - xv)/tau];\nF = A*PL;\nend\n'], ...
    a.A, a.alpha, a.beta, a.gamma, a.Ps, a.kappa, a.Kv, a.tau));
setScript([mdl '/AFC (50 Hz)'], sprintf([ ...
    'function u = fcn(t, x)\n' ...
    'mu0 = [%.16g; %.16g; %.16g]; muinf = [%.16g; %.16g; %.16g];\n' ...
    'al = [%.16g; %.16g; %.16g]; k = [%.16g; %.16g; %.16g];\n' ...
    'mu = (mu0 - muinf).*exp(-al*t) + muinf;\n' ...
    'src = [x(1); x(2); x(5)]; prev = 0; v = 0;\n' ...
    'for i = 1:3\n  z = min(max((src(i) - prev)/mu(i), -1 + 1e-8), 1 - 1e-8);\n' ...
    '  v = -k(i)*0.5*log((1 + z)/(1 - z)); prev = v;\nend\nu = v;\nend\n'], C.mu0, C.muinf, C.alpha, C.k));

% ---- physical connections ----------------------------------------------
ph = @(b) get_param([mdl '/' b], 'PortHandles');
pc = @(b1, side1, i1, b2, side2, i2) add_line(mdl, ph(b1).(side1)(i1), ph(b2).(side2)(i2), 'autorouting', 'on');
expectPorts(mdl, 'Road joint', 2, 3);         % L: B, pz-in   R: F, p, v
expectPorts(mdl, 'Tyre joint', 1, 3);         % L: B          R: F, p, v
expectPorts(mdl, 'Suspension joint', 2, 3);   % L: B, fz-in   R: F, p, v
pc('World', 'RConn', 1, 'Road joint', 'LConn', 1);
pc('Solver Configuration', 'RConn', 1, 'World', 'RConn', 1);
pc('Mechanism Configuration', 'RConn', 1, 'World', 'RConn', 1);
pc('Road joint', 'RConn', 1, 'Road platform', 'RConn', 1);
pc('Road platform', 'RConn', 1, 'Tyre joint', 'LConn', 1);
pc('Tyre joint', 'RConn', 1, 'Wheel carrier', 'RConn', 1);
pc('Wheel carrier', 'RConn', 1, 'Suspension joint', 'LConn', 1);
pc('Suspension joint', 'RConn', 1, 'Body', 'RConn', 1);
pc('zr to PS', 'RConn', 1, 'Road joint', 'LConn', 2);
pc('F to PS', 'RConn', 1, 'Suspension joint', 'LConn', 2);
pc('Road joint', 'RConn', 2, 'q_road', 'LConn', 1);
pc('Road joint', 'RConn', 3, 'v_road', 'LConn', 1);
pc('Tyre joint', 'RConn', 2, 'q_tyre', 'LConn', 1);
pc('Tyre joint', 'RConn', 3, 'v_tyre', 'LConn', 1);
pc('Suspension joint', 'RConn', 2, 'q_susp', 'LConn', 1);
pc('Suspension joint', 'RConn', 3, 'v_susp', 'LConn', 1);
% ---- signal connections ---------------------------------------------------
L = @(s, d) add_line(mdl, s, d, 'autorouting', 'on');
L('Clock/1', 'Road/1'); L('Road/1', 'zr to PS/1'); L('Road/2', 'zr to PS/2'); L('Road/3', 'zr to PS/3');
L('q_road/1', 'States from joints/1'); L('v_road/1', 'States from joints/2');
L('q_tyre/1', 'States from joints/3'); L('v_tyre/1', 'States from joints/4');
L('q_susp/1', 'States from joints/5'); L('v_susp/1', 'States from joints/6');
L('Valve states/1', 'States from joints/7');
L('Valve states/1', 'Servo valve + cylinder/1');
L('Valve rail/1', 'Servo valve + cylinder/2');
L('v_susp/1', 'Servo valve + cylinder/3');
L('Servo valve + cylinder/1', 'Valve states/1');
L('Servo valve + cylinder/2', 'F to PS/1');
L('Digital Clock/1', 'AFC (50 Hz)/1');
L('States from joints/1', 'Sample x/1'); L('Sample x/1', 'AFC (50 Hz)/2');
L('AFC (50 Hz)/1', 'Valve rail/1'); L('Valve rail/1', 'u log/1');
L('States from joints/1', 'x log/1');

set_param(mdl, 'SimulationCommand', 'update');
save_system(mdl, modelFile);
clear cleanupModel;
if usejava('desktop')
    open_system(mdl);   % show the diagram only in an interactive desktop session
end
fprintf('Built %s. Run run_phase2b to compare it with qc_simulate.\n', modelFile);
end

% ======================================================================
function setp(blk, pattern, value, optional)
%SETP Set the first dialog parameter whose name matches the regexp.
if nargin < 4, optional = false; end
names = fieldnames(get_param(blk, 'DialogParameters'));
hit = names(~cellfun(@isempty, regexp(names, pattern, 'once')));
if isempty(hit)
    if optional
        return;
    end
    error('build_quarter_car_multibody:param', ...
        'No parameter matching "%s" on %s.\nAvailable: %s', pattern, blk, strjoin(names', ', '));
end
set_param(blk, hit{1}, value);
end

function setScript(blockPath, code)
ch = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', blockPath);
ch.Script = code;
end

function expectPorts(mdl, b, nl, nr)
p = get_param([mdl '/' b], 'PortHandles');
if numel(p.LConn) ~= nl || numel(p.RConn) ~= nr
    error('build_quarter_car_multibody:ports', ...
        '%s has %d left / %d right physical ports, expected %d / %d. Please paste this message back.', ...
        b, numel(p.LConn), numel(p.RConn), nl, nr);
end
end
