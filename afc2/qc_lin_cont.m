function Lin = qc_lin_cont(P, passive)
%QC_LIN_CONT Continuous-time linearisation of the plant at the origin.
%   Lin = qc_lin_cont(P) returns  xdot = A x + B u + E zr  (bt = 0 assumed
%   for E; a nonzero bt adds a zrdot input in Ezd), plus output rows for
%   body displacement, body acceleration, suspension travel and tire
%   deflection. States follow qc_plant_rhs; the spool state is dropped
%   when the model has no spool dynamics.
%   Lin.Cz = z_s row,  Lin.Ca/Da = body acceleration (= Ca x + Da u + Ea zr)
if nargin < 2
    passive = false;
end
useSpool = strcmp(P.act.model, 'valve') && P.act.tau > 0;
n = 5 + useSpool;
x0 = zeros(6, 1);
f0 = qc_plant_rhs(P, x0, 0, 0, 0, passive);
del = [1e-6 1e-6 1e-6 1e-6 1e-6 1e-9];
A = zeros(n);
for i = 1:n
    dx = x0;
    dx(i) = del(i);
    fp = qc_plant_rhs(P, dx, 0, 0, 0, passive);
    fm = qc_plant_rhs(P, -dx, 0, 0, 0, passive);
    A(:, i) = (fp(1:n) - fm(1:n)) / (2 * del(i));
end
du = 1e-3;
fp = qc_plant_rhs(P, x0, du, 0, 0, passive);
fm = qc_plant_rhs(P, x0, -du, 0, 0, passive);
B = (fp(1:n) - fm(1:n)) / (2 * du);
dz = 1e-6;
fp = qc_plant_rhs(P, x0, 0, dz, 0, passive);
fm = qc_plant_rhs(P, x0, 0, -dz, 0, passive);
E = (fp(1:n) - fm(1:n)) / (2 * dz);
fp = qc_plant_rhs(P, x0, 0, 0, dz, passive);
fm = qc_plant_rhs(P, x0, 0, 0, -dz, passive);
Ezd = (fp(1:n) - fm(1:n)) / (2 * dz);
Lin.A = A;
Lin.B = B;
Lin.E = E;
Lin.Ezd = Ezd;
Lin.n = n;
Lin.Cz = [1, zeros(1, n - 1)];
Lin.Ca = A(2, :);
Lin.Da = B(2);
Lin.Ea = E(2);
Lin.Ctravel = [1, 0, -1, zeros(1, n - 3)];
Lin.Ctire = [0, 0, 1, zeros(1, n - 3)];   % minus zr gives tire deflection
Lin.residual = norm(f0);
end
