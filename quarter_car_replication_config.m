function cfg = quarter_car_replication_config()
% Published paper settings plus explicitly configurable plant assumptions.
cfg.sim.duration_s = 20;
cfg.sim.sample_time_s = 1 / 50;
cfg.sim.integration_substeps = 4;
cfg.sim.initial_state = zeros(5, 1);
cfg.sim.voltage_limit_V = 5;

% Representative quarter-car constants from Jin et al. (2019):
% ms=320 kg, mu=40 kg, ks=18 kN/m, kt=200 kN/m, bs=1 kN s/m.
cfg.plant.ms_kg = 320;
cfg.plant.mu_kg = 40;
cfg.plant.ks_N_m = 18000;
cfg.plant.ksn_N_m3 = 0;
cfg.plant.bs_Ns_m = 1000;
cfg.plant.kt_N_m = 200000;
cfg.plant.bt_Ns_m = 0;

% Area, beta, and supply pressure are illustrative values from a separate
% published suspension model (Abdulzahra and Abdalla, 2019), not the target rig.
% x5 = kappa * PL; force = area * x5 / kappa.
cfg.plant.area_m2 = 3.35e-4;
cfg.plant.kappa = 1e-6;

% Equivalent first-order pressure-state model for missing valve constants:
% x5dot = -beta*x5 - cRel*(zdot_s-zdot_u) + bu*u.
cfg.actuator.model = 'equivalent';
cfg.actuator.beta_per_s = 1;
cfg.actuator.cRel = 0.05;
cfg.actuator.bu_per_s_V = 0.75;
cfg.actuator.pressure_limit_Pa = 10.3425e6;

% Optional nonlinear valve model from the target paper. Populate from
% identified actuator data before selecting model='paper-valve'.
cfg.actuator.a = NaN;
cfg.actuator.gamma = NaN;
cfg.actuator.supply_pressure_Pa = NaN;
cfg.actuator.fluid_density_kg_m3 = NaN;

% Case A PPF and feedback values reported in Na et al. (2022).
cfg.afc.delta = 1;
cfg.afc.delta_bar = 1;
cfg.afc.mu0 = [0.2; 110; 100];
cfg.afc.mu_inf = [0.018; 90; 80];
cfg.afc.alpha = [3; 2; 2];
cfg.afc.k = [25.5; 12; 216];

cfg.backstepping.k = [400; 100; 400];
cfg.pid.kp = 3080;
cfg.pid.ki = 200;
cfg.pid.kd = 200;
end
