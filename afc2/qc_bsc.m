function u = qc_bsc(C, x, P, zr, zrd)
%QC_BSC Model-based backstepping comparator (Na et al. Sec. IV-A, after Ge & Wang).
%   nu1 = x1,           alpha1 = -k1 nu1
%   nu2 = x2 - alpha1,  alpha2 = kappa(-k2 nu2 ms + Fd + Fs + ms alpha1' - ms nu1)/A
%   nu3 = x5 - alpha2
%   u   = (-k3 nu3 - A nu2/(ms kappa) + beta x5 + kappa alpha A (x2 - x4) + alpha2')
%         / (kappa gamma Kv w3),   w3 = sqrt(Ps - sgn(u) P_L)
%   Fd, Fs and the actuator model are assumed known (as stated in the paper);
%   the spool lag is NOT known to the controller. The paper prints
%   kappa*a*A*x2; the relative velocity (x2 - x4) is used here to match eq. (7).
%
%   For the v1 'equivalent' actuator, the v1 formula (no cross terms) is kept
%   so that committed v1 results are reproduced exactly.
m = P.mech;
a = P.act;
k1 = C.k(1); k2 = C.k(2); k3 = C.k(3);
dx = qc_plant_rhs(P, x, 0, zr, zrd, false);   % accelerations do not depend on u
d = x(1) - x(3);
Fs = m.ks * d + m.ksn * d^3;
Fd = m.bs * (x(2) - x(4));
Fdd = m.bs * (dx(2) - dx(4));
Fsd = (m.ks + 3 * m.ksn * d^2) * (x(2) - x(4));

nu1 = x(1);
alpha1 = -k1 * nu1;
alpha1d = -k1 * x(2);
alpha1dd = -k1 * dx(2);
nu2 = x(2) - alpha1;
nu2d = dx(2) - alpha1d;

switch a.model
    case 'equivalent'
        alpha2 = a.kappa / a.A * (-m.ms * k2 * nu2 + Fd + Fs + m.ms * alpha1d);
        alpha2d = a.kappa / a.A * (-m.ms * k2 * nu2d + Fdd + Fsd + m.ms * alpha1dd);
        nu3 = x(5) - alpha2;
        drift = -a.beta * x(5) - a.cRel * (x(2) - x(4));
        u = (-k3 * nu3 + alpha2d - drift) / a.bu;
    case 'valve'
        alpha2 = a.kappa * (-k2 * nu2 * m.ms + Fd + Fs + m.ms * alpha1d - m.ms * nu1) / a.A;
        alpha2d = a.kappa * (-k2 * nu2d * m.ms + Fdd + Fsd + m.ms * alpha1dd - m.ms * x(2)) / a.A;
        nu3 = x(5) - alpha2;
        num = -k3 * nu3 - a.A * nu2 / (m.ms * a.kappa) + a.beta * x(5) + ...
            a.kappa * a.alpha * a.A * (x(2) - x(4)) + alpha2d;
        s = sign(num);
        if s == 0
            s = 1;
        end
        PL = x(5) / a.kappa;
        w3 = sqrt(max(a.Ps - s * PL, 1));
        u = num / (a.kappa * a.gamma * a.Kv * w3);
    otherwise
        error('qc_bsc:model', 'Unknown actuator model.');
end
end
