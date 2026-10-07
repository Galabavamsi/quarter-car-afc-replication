function C = qc_ctrl(type, variant)
%QC_CTRL Controller presets from Na et al. (2022), Section IV.
%
%   C = qc_ctrl('afc', 'A')   Case A (10-peak road): mu1=(0.2-0.018)e^-3t+0.018,
%                             mu2=(110-90)e^-2t+90, mu3=(100-80)e^-2t+80,
%                             k = [25.5 12 216]
%   C = qc_ctrl('afc', 'B')   retuned for slower roads (3-9 peaks):
%                             mu1=(0.12-0.018)e^-2t+0.018, mu2=(80-40)e^-2t+40,
%                             mu3=(50-20)e^-4t+20, k = [12 5 146]
%   C = qc_ctrl('bsc', 'A'|'B')  k = [400 100 400] | [400 100 260]
%   C = qc_ctrl('pid', 'A'|'B')  [kP kI kD] = [3080 200 200] | [1000 200 100]
%   C = qc_ctrl('passive')
%   C = qc_ctrl('afc_aw', 'A')  extension: AFC with saturation-driven
%                               envelope relaxation (anti-windup)
%   C = qc_ctrl('skyhook')      skyhook damper + inner pressure loop
%   LQR is built with qc_lqr_design(P, weights) because it needs the plant.
%
%   delta = delta_bar = 1 is a project inference (not printed in the paper).
if nargin < 2 || isempty(variant)
    variant = 'A';
end
variant = upper(variant);
C.type = lower(type);
C.variant = variant;
switch C.type
    case 'afc'
        C.delta = 1;
        C.dbar = 1;
        C.guard = 1e-8;   % numerical open-interval guard for the log
        switch variant
            case 'A'
                C.mu0 = [0.2; 110; 100];
                C.muinf = [0.018; 90; 80];
                C.alpha = [3; 2; 2];
                C.k = [25.5; 12; 216];
            case 'B'
                C.mu0 = [0.12; 80; 50];
                C.muinf = [0.018; 40; 20];
                C.alpha = [2; 2; 4];
                C.k = [12; 5; 146];
            otherwise
                error('qc_ctrl:variant', 'AFC variant must be A or B.');
        end
        C.label = sprintf('AFC (Case %s)', variant);
    case 'bsc'
        if strcmp(variant, 'A')
            C.k = [400; 100; 400];
        else
            C.k = [400; 100; 260];
        end
        C.label = sprintf('BSC (Case %s)', variant);
    case 'pid'
        if strcmp(variant, 'A')
            C.kp = 3080; C.ki = 200; C.kd = 200;
        else
            C.kp = 1000; C.ki = 200; C.kd = 100;
        end
        C.I = 0;          % integrator state (conditional integration)
        C.label = sprintf('PID (Case %s)', variant);
    case 'afc_aw'
        % Extension (not in the paper): AFC whose envelopes widen while the
        % valve is saturated, mu_i,eff = mu_i (1 + rho),
        %   rho' = -ell*rho + gam * |u_raw - sat(u_raw)| / Vmax,
        % so the log barrier never sees an error it cannot act on.
        C = qc_ctrl('afc', variant);
        C.type = 'afc_aw';
        C.rho = 0;
        C.ell = 0.5;     % 1/s, recovery rate back to the nominal envelope
        C.gam = 5;       % 1/s, widening rate per unit normalised excess
        C.label = sprintf('AFC-AW (Case %s)', variant);
    case 'skyhook'
        % Classic skyhook comparator with the same inner pressure loop gain
        % as AFC Case A near the origin (k3/mu3_inf = 2.7 V per unit x5).
        C.csky = 4000;   % N s/m, skyhook damper
        C.ksky = 0;      % N/m, skyhook spring (0 = pure Karnopp skyhook)
        C.kp = 2.7;      % V per unit of x5 error
        C.label = 'Skyhook';
    case 'passive'
        C.label = 'Passive';
    otherwise
        error('qc_ctrl:type', 'Unknown controller "%s".', type);
end
end
