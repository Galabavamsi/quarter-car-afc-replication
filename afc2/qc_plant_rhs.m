function [dx, acc, F] = qc_plant_rhs(P, x, u, zr, zrd, passive)
%QC_PLANT_RHS Quarter-car + electro-hydraulic actuator dynamics.
%   [dx, acc, F] = qc_plant_rhs(P, x, u, zr, zrd, passive)
%   x  = [z_s; zdot_s; z_u; zdot_u; x5 = kappa*P_L; x_v]
%   u  = valve command (V), already saturated by the caller
%   zr, zrd = road displacement and velocity
%   passive = true removes the actuator (F = 0), i.e. spring + damper only.
%
%   Mechanics (Na et al. eq. 1/7):
%     ms*zs'' = -Fd - Fs + F
%     mu*zu'' =  Fd + Fs - Ft - Fb - F
%   Actuator, P.act.model:
%     'equivalent' : x5' = -beta*x5 - cRel*(zs'-zu') + bu*u        (v1 surrogate)
%     'valve'      : P_L' = -beta*P_L - alpha*A*(zs'-zu')
%                           + gamma*x_v*sqrt(Ps - sgn(x_v)*P_L)     (eqs. 2-6)
%                    x_v' = (Kv*u - x_v)/tau   (tau = 0 -> x_v = Kv*u)
if nargin < 6
    passive = false;
end
m = P.mech;
a = P.act;

d = x(1) - x(3);
v = x(2) - x(4);
Fs = m.ks * d + m.ksn * d^3;
Fd = m.bs * v;
Ft = m.kt * (x(3) - zr);
Fb = m.bt * (x(4) - zrd);

dx = zeros(6, 1);
if passive
    F = 0;
else
    F = a.A * x(5) / a.kappa;
    switch a.model
        case 'equivalent'
            dx5 = -a.beta * x(5) - a.cRel * v + a.bu * u;
            lim = a.kappa * a.Plim;
            if (x(5) >= lim && dx5 > 0) || (x(5) <= -lim && dx5 < 0)
                dx5 = 0;
            end
            dx(5) = dx5;
        case 'valve'
            PL = x(5) / a.kappa;
            if a.tau > 0
                xv = x(6);
                dx(6) = (a.Kv * u - x(6)) / a.tau;
            else
                xv = a.Kv * u;
            end
            dP = a.Ps - sign(xv) * PL;
            dPL = -a.beta * PL - a.alpha * a.A * v + ...
                a.gamma * xv * sqrt(max(dP, 0));
            dx(5) = a.kappa * dPL;
        otherwise
            error('qc_plant_rhs:model', 'Unknown actuator model "%s".', a.model);
    end
end

acc = (-Fd - Fs + F) / m.ms;
dx(1) = x(2);
dx(2) = acc;
dx(3) = x(4);
dx(4) = (Fd + Fs - Ft - Fb - F) / m.mu;
end
