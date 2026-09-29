# Quarter-car AFC: paper analysis and MATLAB/Simulink reproduction

This is a course-project reproduction of Jing Na et al., ["Active Suspension Control of Quarter-Car System With Experimental Validation"](https://doi.org/10.1109/TSMC.2021.3103807), *IEEE Transactions on Systems, Man, and Cybernetics: Systems* 52(8), 4714-4726 (2022).

## Deliverables

- [Research report](deliverables/QuarterCar_AFC_Research_Report.pdf) and its [LaTeX source](QuarterCar_AFC_Research_Report.tex)
- [Presentation](deliverables/QuarterCar_AFC_Presentation.pdf) and its [LaTeX source](QuarterCar_AFC_Presentation.tex)
- [Editable Simulink model](QuarterCar_AFC_Replication.slx)
- [Publication figures and genuine Simulink/MATLAB captures](paper_assets)
- [Presentation notes](Presentation_Notes.md)

The report and presentation label source-paper measurements separately from project simulations. The publisher's licensed PDF is not redistributed here.

## Reproduce

Requires MATLAB R2024b with Simulink for the tested workflow. From this repository directory in MATLAB:

```matlab
results = run_quarter_car_replication();
generate_research_assets(true);
open_system('QuarterCar_AFC_Replication');
```

`run_quarter_car_replication` regenerates `quarter_car_replication_results.mat`, principal plots, and controller metrics. `generate_research_assets(true)` exports the paper figures, captures the actual Simulink model, runs the Simulink cross-check, and writes `paper_assets/simulink_crosscheck.mat`. To rebuild the model after changing its builder, run `build_quarter_car_simulink(true)`; this overwrites the existing `.slx`, so save any manual edits separately first.

The LaTeX files use ordinary `pdflatex` with TikZ, `siunitx`, and Beamer. Run `pdflatex` twice on each source from the repository root so cross-references resolve. Their figures are relative to this directory.

## What is reproduced

The five-state quarter-car plant includes sprung and unsprung dynamics, suspension and tire forces, and a scaled hydraulic pressure state. The controller implements the paper's approximation-free control (AFC) recursion with prescribed performance functions. The comparative MATLAB script evaluates AFC, model-based backstepping, PID, and passive suspension for the paper's 10-peak sinusoidal road case, and runs AFC through all eight published sinusoidal road conditions. The sampled controller runs at 50 Hz with a +/-5 V command limit. MATLAB RK4 integration and a separate Simulink `ode4` implementation agree to a maximum body-displacement difference of about 2.10e-8 m on the main case.

## What is *not* reproduced

The target paper does not release the complete identified test-rig/valve constants or raw measured time series. The default plant therefore uses representative mechanical values from [Jin et al. (2019)](https://doi.org/10.1155/2019/1783850), illustrative actuator values motivated by [Abdulzahra and Abdalla (2019)](https://doi.org/10.5815/ijisa.2019.12.01), and openly configurable pressure-state assumptions in [`quarter_car_replication_config.m`](quarter_car_replication_config.m). It also uses simulated full-state feedback rather than the paper's sensor/observer chain. The symmetric PPF multiplier setting is inferred, not published numerically. The road sweep holds AFC gains fixed, whereas the paper retunes some cases.

This distinction matters in the results: the representative Case 10 simulation improves AFC displacement IAE over passive suspension (0.4849 versus 0.6709 m s), but the command saturates for 85.4% of samples and the body trajectory exceeds the assumed displacement envelope by up to 0.01885 m. The project does **not** claim to reproduce the paper's hardware ranking or validate its PPF guarantee under these surrogate actuator assumptions.

## File map

| File | Role |
| --- | --- |
| `quarter_car_replication_config.m` | Parameters and published AFC settings |
| `run_quarter_car_replication.m` | Sampled MATLAB simulation, comparators, metrics |
| `build_quarter_car_simulink.m` | Editable Simulink model generator |
| `generate_research_assets.m` | High-resolution plots, model capture, cross-check |
| `paper_assets/` | MATLAB-exported figures and real model/figure captures |
| `deliverables/` | Final report and presentation PDFs |

To approach a quantitative replication, identify the original hydraulic actuator and mechanical constants, obtain the road and measured response samples, reproduce the force observer/sensor processing, and apply the source paper's per-case retuning before comparing aligned trajectories.
