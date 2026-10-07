function [u, C, info] = qc_controller(C, t, x, P, zr, zrd)
%QC_CONTROLLER Evaluate one 50 Hz controller sample. Returns the raw
%   (unsaturated) command u, the updated controller struct (for stateful
%   controllers such as PID) and an info struct with internal signals.
info = struct();
switch C.type
    case 'afc'
        [u, info] = qc_afc(C, t, x);
    case 'bsc'
        u = qc_bsc(C, x, P, zr, zrd);
    case 'pid'
        % error = 0 - z_s ; derivative on measurement ; conditional
        % integration (integrate only when unsaturated or when it helps unwind)
        e = -x(1);
        Ic = C.I + P.sim.Ts * e;
        uc = C.kp * e + C.ki * Ic - C.kd * x(2);
        if abs(uc) <= P.sim.Vmax || sign(uc) ~= sign(e)
            C.I = Ic;
        end
        u = C.kp * e + C.ki * C.I - C.kd * x(2);
    case 'afc_aw'
        [u, info] = qc_afc(C, t, x);
        excess = max(abs(u) - P.sim.Vmax, 0) / P.sim.Vmax;
        C.rho = max(0, C.rho + P.sim.Ts * (-C.ell * C.rho + C.gam * excess));
        info.rho = C.rho;
    case 'lqr'
        u = -C.K * x(1:C.n);
    case 'skyhook'
        a = P.act;
        Fref = -C.csky * x(2) - C.ksky * x(1);
        u = -C.kp * (x(5) - a.kappa * Fref / a.A);
    case 'passive'
        u = 0;
    otherwise
        error('qc_controller:type', 'Unknown controller "%s".', C.type);
end
end
