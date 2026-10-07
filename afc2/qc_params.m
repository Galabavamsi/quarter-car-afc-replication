function P = qc_params(name)
%QC_PARAMS Plant and simulation parameter sets for the afc2 library.
%
%   P = qc_params('v1-equivalent')
%       The original project model (Jin et al. masses, first-order
%       "equivalent" pressure state). Reproduces the committed v1 results.
%
%   P = qc_params('alleyne')
%       Alleyne & Hedrick (1995) electro-hydraulic quarter car with the
%       nonlinear servo-valve flow law of Na et al. eqs. (2)-(6) and
%       first-order spool dynamics, literature constants only. The two
%       quantities the literature does not fix for a 50 Hz voltage-driven
%       rig (valve gain Kv in m/V and the pressure scale kappa) are set to
%       the calibrated values below.
%
%   P = qc_params('alleyne-cal')
%       Same plant with the lumped valve/sensing lag tau calibrated so that
%       the paper's *unchanged* Case A AFC reproduces the paper's reported
%       Case 10 AFC indices (Table II) and voltage level (Fig. 13).
%       See run_phase1.m, section 2, for the calibration grid.
%
%   P = qc_params('alleyne-joint')
%       Same plant with (kappa, Kv, tau) fitted jointly to two training
%       roads that use different paper gain sets (Case 10/A and Case 6/B).
%       Used to test whether ANY member of this plant family explains the
%       paper's data under both gain sets (run_phase1.m, section 4).
%
%   State vector (all models): x = [z_s; zdot_s; z_u; zdot_u; x5; x_v]
%       x5 = kappa * P_L (scaled load pressure), x_v = spool displacement.
%
% Provenance flags are stored in P.provenance so figures/tables can print them.

if nargin < 1 || isempty(name)
    name = 'alleyne-cal';
end
P.name = lower(name);
P.g = 9.81;

% ---- sampled-data simulation settings (paper: 50 Hz, +/-5 V) -------------
P.sim.T = 20;          % s, paper evaluates indices over 0-20 s
P.sim.Ts = 0.02;       % s, 50 Hz controller
P.sim.h = 0.001;       % s, RK4 integration step inside each sample
P.sim.Vmax = 5;        % V, valve command rail
P.sim.x0 = zeros(6, 1);

switch P.name
    case 'v1-equivalent'
        P.mech = struct('ms', 320, 'mu', 40, 'ks', 18000, 'ksn', 0, ...
            'bs', 1000, 'kt', 200000, 'bt', 0);
        P.act = struct('model', 'equivalent', 'A', 3.35e-4, 'kappa', 1e-6, ...
            'beta', 1, 'cRel', 0.05, 'bu', 0.75, 'Plim', 10.3425e6, ...
            'alpha', NaN, 'gamma', NaN, 'Ps', NaN, 'Kv', NaN, 'tau', 0);
        P.sim.h = 0.005;   % v1 used 4 RK4 substeps per 20 ms sample
        P.provenance = ['Mechanics: Jin et al. (2019). Actuator: assumed ' ...
            'first-order equivalent pressure model (v1 project assumption).'];

    case {'alleyne', 'alleyne-cal', 'alleyne-joint'}
        % Alleyne & Hedrick, IEEE TCST 3(1), 1995 - widely reused constants.
        P.mech = struct('ms', 290, 'mu', 59, 'ks', 16812, 'ksn', 0, ...
            'bs', 1000, 'kt', 190000, 'bt', 0);
        P.act = struct('model', 'valve', ...
            'A', 3.35e-4, ...      % m^2 piston area
            'alpha', 4.515e13, ... % N/m^5, = 4*beta_e/V_t
            'beta', 1, ...         % 1/s, leakage term
            'gamma', 1.545e9, ...  % N/(m^(5/2) kg^(1/2)), valve flow gain
            'Ps', 10342500, ...    % Pa, supply pressure
            'kappa', 1e-6, ...     % x5 = kappa*P_L (paper's scaling, value assumed)
            'Kv', 5e-4, ...        % m/V, voltage -> spool (calibrated)
            'tau', 0.003, ...      % s, spool time constant (Alleyne-Hedrick)
            'cRel', NaN, 'bu', NaN, 'Plim', 10342500);
        P.provenance = ['Mechanics + hydraulics: Alleyne & Hedrick (1995). ' ...
            'Kv, kappa: assumed scale factors.'];
        if strcmp(P.name, 'alleyne-cal')
            P.act.tau = 1/30;  % s, lumped valve + sensing lag (calibrated)
            P.provenance = ['Mechanics + hydraulics: Alleyne & Hedrick (1995). ' ...
                'Kv = 5e-4 m/V and lumped lag tau = 1/30 s calibrated to the ' ...
                'paper''s Case 10 AFC indices; kappa = 1e-6 assumed.'];
        elseif strcmp(P.name, 'alleyne-joint')
            % Nelder-Mead fit of (kappa, Kv, tau) to AFC IAE and voltage RMS
            % on two TRAINING roads (Case 10 / gains A, Case 6 / gains B);
            % the other six roads are held out (see run_phase1 section 4).
            P.act.kappa = 8.78e-7;
            P.act.Kv = 4.06e-4;
            P.act.tau = 0.0532;
            P.provenance = ['Mechanics + hydraulics: Alleyne & Hedrick (1995). ' ...
                'kappa, Kv, tau jointly fitted to paper Case 10 (A) and Case 6 (B) AFC data.'];
        end

    otherwise
        error('qc_params:unknown', 'Unknown parameter set "%s".', name);
end
end
