# afc2: quarter-car AFC library (v2)

Toolbox-free MATLAB functions, also tested in GNU Octave 8.4. The v1 scripts in the repository root are left unchanged. `qc_params('v1-equivalent')` reproduces the v1 numbers exactly (AFC IAE 0.4849, 85.4% saturation).

| File | Purpose |
| --- | --- |
| `qc_params.m` | Plant sets: `v1-equivalent`, `alleyne` (literature), `alleyne-cal` (Case-10 calibration), `alleyne-joint` (2-road fit) |
| `qc_plant_rhs.m` | Quarter car + actuator: v1 first-order surrogate, or the paper's nonlinear servo-valve law (eqs. 2-6) with spool lag |
| `qc_road.m` | Paper road cases 3-10, sine, (1-cos) bump, ISO 8608 random road (closed-form in t) |
| `qc_ctrl.m` | Paper controller presets: AFC/BSC/PID Case A and Case B, passive |
| `qc_afc.m` | AFC law, eqs. (9)-(20); logs unclipped zeta, eps, mu for the stability plots |
| `qc_bsc.m` | Model-based backstepping comparator (paper Sec. IV-A) |
| `qc_controller.m` | Dispatcher (PID with conditional integration lives here) |
| `qc_simulate.m` | 50 Hz ZOH controller + RK4 plant, voltage rail, divergence guard |
| `qc_metrics.m` | IAE/ITAE/ITSE, acc RMS/max, voltage RMS/max, saturation, bound violation, travel, tire load, PSD bands |
| `qc_psd.m` | Welch PSD without the Signal Processing Toolbox |
| `qc_linearize.m` | Jacobian of the sampled closed-loop map: eigenvalues, spectral radius, equivalent linear gain |
| `qc_paper_ref.m` | Paper numbers: printed (Table II/III, Fig. 7/8) vs read off plots (Figs. 12-13, approx.) |
| `qc_lin_cont.m` | Continuous linearisation `xdot = A x + B u + E zr` with output rows (displacement, acceleration, travel) |
| `qc_lqr_design.m` | Discrete LQR comparator (ZOH model, Riccati by iteration, no toolbox) |
| `qc_kalman_design.m` | Steady-state Kalman filter from the paper's two sensors (z_s, a_s); stands in for the paper's ESO |
| `qc_sine_gain.m` | Empirical frequency response of the sampled nonlinear loop (sine roads, LS fit) |
| `qc_randn.m`, `qc_kronecker.m` | Reproducible noise and Monte Carlo samples (identical in MATLAB and Octave) |

Controllers in `qc_ctrl`: `afc` (A/B), `afc_aw` (extension: saturation-driven envelope relaxation), `bsc`, `pid`, `skyhook`, `passive`. LQR comes from `qc_lqr_design`.
Sensing modes in `qc_simulate(..., struct('sensing', struct('mode', m)))`: `ideal`, `diff`, `accint`, `kalman`.
| `qc_style.m`, `qc_save_fig.m`, `qc_write_csv.m` | Plot style, PDF/PNG export, CSV writer |

## Minimal use

```matlab
addpath('afc2');
P = qc_params('alleyne-cal');
run = qc_simulate(P, qc_road('case', 10), qc_ctrl('afc', 'A'));
run.metrics            % IAE ~0.129, u_rms ~0.50 V, no saturation, bound kept
L = qc_linearize(P, qc_ctrl('afc', 'A'));   % L.rho > 1: locally unstable at the origin
```

## Evidence labels used in every output
- **printed**: a number printed in Na et al. (Table II/III, Figs. 7-8).
- **approx.**: read off a paper plot by eye (Figs. 12-13).
- **calibrated**: a free scale factor fitted to printed or approximate paper data.
- **assumed**: a value that no source fixes.
