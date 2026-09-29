# Quarter-car active suspension: AFC paper reproduction

**Status:** runnable, documented **method-level reproduction** for an Advanced Control Theory paper presentation. It is **not** a reconstruction of the authors' hardware experiment. This repository contains the MATLAB simulation, editable Simulink model, source and PDF versions of the report and slides, and every current project-generated figure PDF.

**Source paper:** J. Na et al., ["Active Suspension Control of Quarter-Car System With Experimental Validation"](https://doi.org/10.1109/TSMC.2021.3103807), *IEEE Transactions on Systems, Man, and Cybernetics: Systems*, 52(8), 4714-4726, 2022 ([IEEE Xplore](https://ieeexplore.ieee.org/document/9530282)). In this project **AFC means approximation-free control**; **PPF means prescribed performance function**.

![MATLAB response comparison for the 10-peak road case](paper_assets/response_comparison.png)

> **Read this first:** Paper-reported measurements and our representative-model simulations are different evidence. In particular, the source paper's fixed-waveform hardware acceleration numbers must **not** be compared numerically with our 10-peak sinusoidal simulation.

## PDF library

All **current, project-generated PDFs** are in this public repository and linked individually below. Figure PDFs are vector exports from MATLAB; PNG copies are also in [`paper_assets/`](paper_assets/).

| PDF | What it contains |
| --- | --- |
| [Research report](deliverables/QuarterCar_AFC_Research_Report.pdf) | 7-page paper-style analysis, equations, original mechanical schematic, assumptions, results, limitations, references |
| [Presentation](deliverables/QuarterCar_AFC_Presentation.pdf) | 18-slide course presentation with real Simulink/MATLAB captures and findings |
| [Road and body response](paper_assets/response_comparison.pdf) | Case 10 road input and AFC/BSC/PID/passive displacement traces |
| [PPF audit](paper_assets/ppf_audit.pdf) | AFC trajectory, prescribed bounds, and normalized violation ratio |
| [Acceleration and control](paper_assets/acceleration_control.pdf) | Body acceleration and applied voltage against the +/-5 V rails |
| [Controller metrics](paper_assets/metric_comparison.pdf) | IAE, ITAE, ITSE, and acceleration RMS for four controller cases |
| [Eight-road sweep](paper_assets/road_sweep.pdf) | AFC error indices over the eight published sinusoidal input pairs, with fixed tuning |
| [MATLAB/Simulink cross-check](paper_assets/implementation_crosscheck.pdf) | Overlaid body displacement from independent numerical implementations |

The older PDF under `report-build/` is a superseded local draft, **not** a current deliverable. The publisher-downloaded source PDF carries a restricted-use notice and is **not redistributed** in this public repository; use the [IEEE-hosted PDF](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9530282) or your institution's access instead.

## Five-minute takeover

1. Read the [report](deliverables/QuarterCar_AFC_Research_Report.pdf) for the scientific argument, then the [slides](deliverables/QuarterCar_AFC_Presentation.pdf) and [speaker notes](Presentation_Notes.md) for the presentation story.
2. Open [`quarter_car_replication_config.m`](quarter_car_replication_config.m). It is the single place to inspect the published AFC settings versus the representative/assumed plant settings.
3. Run the MATLAB commands below from the repository root; inspect the printed metrics, generated figures, and [`paper_assets/simulink_crosscheck.mat`](paper_assets/simulink_crosscheck.mat).
4. Open [`QuarterCar_AFC_Replication.slx`](QuarterCar_AFC_Replication.slx) in Simulink. The diagram implements AFC only; the MATLAB script supplies the BSC, PID, passive, and road-sweep comparisons.
5. Before changing a figure or claim, check the **Evidence boundary** and **Next work** sections below. Keep paper measurements, assumed values, and new measurements explicitly labeled.

## Run and regenerate

**Tested environment:** MATLAB R2024b Update 2 on Windows with Simulink. The model uses MATLAB Function blocks. The report and slides were built with `pdflatex`/MiKTeX; an equivalent TeX Live installation should also work. Run commands from the repository root because the MATLAB scripts use the current directory for input/output files.

```matlab
% MATLAB Command Window, with this repository as the current folder
results = run_quarter_car_replication();
generate_research_assets(true);
open_system('QuarterCar_AFC_Replication');
```

`run_quarter_car_replication` saves [`quarter_car_replication_results.mat`](quarter_car_replication_results.mat), prints metrics, and writes quick-look PNGs to the ignored `replication_figures/` directory. `generate_research_assets(true)` rebuilds the vector PDF/PNG plots, captures the actual Simulink model and MATLAB figure, runs the model cross-check, and saves [`paper_assets/simulink_crosscheck.mat`](paper_assets/simulink_crosscheck.mat). Set its argument to `false` to regenerate figures without rerunning Simulink; the previously generated cross-check file is then left as-is.

**Changing parameters:** edit [`quarter_car_replication_config.m`](quarter_car_replication_config.m), then run `build_quarter_car_simulink(true)`, `run_quarter_car_replication()`, and `generate_research_assets(true)` in that order. The model builder reads the configuration function itself and embeds those values in the `.slx`; changing only a workspace `cfg` will **not** update the model. `build_quarter_car_simulink(true)` overwrites the tracked `.slx` and refuses to overwrite a loaded model with unsaved edits, so preserve any manual Simulink changes first.

**Sanity checks with the committed defaults:** Case 10 AFC IAE is about `0.4849 m s`, AFC saturation fraction about `85.4%`, and maximum displacement-envelope violation about `0.01885 m`. The MATLAB/Simulink maximum body-displacement difference is `2.09655e-8 m` in the tested environment; modest numerical variation across releases/platforms is possible. A successful cross-check establishes **implementation agreement for the chosen model**, not agreement with the physical rig.

To rebuild the PDF deliverables after editing LaTeX, run `pdflatex` **twice per source** from the repository root so citations and figure references resolve:

```powershell
pdflatex -interaction=nonstopmode -halt-on-error -output-directory=deliverables QuarterCar_AFC_Research_Report.tex
pdflatex -interaction=nonstopmode -halt-on-error -output-directory=deliverables QuarterCar_AFC_Research_Report.tex
pdflatex -interaction=nonstopmode -halt-on-error -output-directory=deliverables QuarterCar_AFC_Presentation.tex
pdflatex -interaction=nonstopmode -halt-on-error -output-directory=deliverables QuarterCar_AFC_Presentation.tex
```

The plots use MATLAB's [`exportgraphics`](https://www.mathworks.com/help/matlab/ref/exportgraphics.html) vector-PDF export; the model capture uses Simulink's documented [programmatic model print](https://www.mathworks.com/help/simulink/ug/print-from-the-matlab-command-line.html). The quarter-car schematic in the report is original TikZ artwork, not a copied paper figure.

## Model and control in one screen

The five states are $x=[z_s,\dot z_s,z_u,\dot z_u,\kappa P_L]^T$: body displacement/velocity, wheel displacement/velocity, and scaled hydraulic load pressure. The actuator force is $F_a=A P_L=A x_5/\kappa$. Suspension/tire force balance is implemented in [`run_quarter_car_replication.m`](run_quarter_car_replication.m); the default pressure state is a configurable **equivalent** model, not the source paper's nonlinear valve equation:

$$\dot x_5=-\beta x_5-c_{\mathrm{rel}}(\dot z_s-\dot z_u)+b_u u,\qquad |u|\leq5\ \mathrm{V}.$$

The paper's AFC is a three-stage recursion. For each stage, the shrinking performance bound is $\mu_i(t)=(\mu_{i0}-\mu_{i\infty})e^{-\alpha_i t}+\mu_{i\infty}$, the normalized error is $\zeta_i=e_i/\mu_i$, and the transformed error is $\epsilon_i=\tfrac12\log[(\delta+\zeta_i)/(\bar\delta-\zeta_i)]$. The controller uses $e_1=x_1$, $e_2=x_2-u_1$, $e_3=x_5-u_2$, with $u_i=-k_i\epsilon_i$ and final command $u=u_3$. The code guards the logarithm's open interval numerically; **that guard does not enforce a violated physical PPF**.

| Quantity | Committed value | Provenance |
| --- | ---: | --- |
| $m_s,m_u$ | 320, 40 kg | Representative values from [Jin et al. (2019)](https://doi.org/10.1155/2019/1783850), not the target rig |
| $k_s,k_t,b_s$ | 18, 200 kN/m; 1 kN s/m | Same separate quarter-car study |
| $k_{sn},b_t$ | 0, 0 | Simplifying assumptions |
| $A,\beta,P_{\max}$ | $3.35\times10^{-4}$ m², 1 s⁻¹, 10.3425 MPa | Illustrative values motivated by [Abdulzahra and Abdalla (2019)](https://doi.org/10.5815/ijisa.2019.12.01); not target-rig measurements |
| $\kappa,c_{\mathrm{rel}},b_u$ | $10^{-6}$, 0.05, 0.75 | Configurable project assumptions |
| AFC $k_1,k_2,k_3$ | 25.5, 12, 216 | Source paper's Case A |
| PPF $\mu_{10}\to\mu_{1\infty}$ | 0.2 -> 0.018 m, $\alpha_1=3$ | Source paper's Case A |
| $\delta,\bar\delta$ | 1, 1 | Inferred symmetric choice; not numerically stated in the source paper |
| Controller sample time; voltage rail | 0.02 s (50 Hz); +/-5 V | Source paper implementation |

The optional `paper-valve` actuator mode in the MATLAB code is **not ready to run with defaults**: its valve/fluid constants are `NaN` until measured or identified values are supplied.

## Experiments and current findings

The main comparison uses the paper's 10-peak sinusoidal road case, $z_r(t)=0.043\sin(2\pi\,0.505t)$ m, over 20 s. The MATLAB program runs AFC, model-based backstepping (BSC), PID, and passive cases, then computes IAE, ITAE, ITSE, acceleration RMS/peak, voltage effort, saturation fraction, and displacement PPF violation. Its AFC sweep uses all eight published amplitude/frequency pairs but **holds Case A gains fixed**, unlike the source paper's retuning of several slower cases.

| Controller | IAE (m s) | ITAE (m s²) | ITSE (m² s²) | Acc. RMS (m/s²) |
| --- | ---: | ---: | ---: | ---: |
| AFC | 0.4849 | 4.8070 | 0.1369 | 0.3894 |
| BSC | 0.5248 | 5.2841 | 0.1686 | 0.3921 |
| PID | **0.4698** | **4.7166** | **0.1309** | **0.3676** |
| Passive | 0.6709 | 6.6983 | 0.2784 | 0.3965 |

These are **project simulation results**, not paper measurements. AFC improves displacement IAE over passive here, but PID narrowly leads under the assumed plant. The raw AFC command exceeds the +/-5 V rail in **85.4%** of samples, and the body trajectory exceeds the assumed symmetric PPF by up to **0.01885 m**. The paper separately reports PPF-compliant measured displacement on its rig and, for its **fixed-waveform** hardware test, AFC acceleration RMS `1.6651 m/s²` and maximum `7.21 m/s²`. Do not put those hardware acceleration numbers into the table above or use them as a same-input validation target.

## Evidence boundary and known gaps

- **Available from the paper:** AFC equations and Case A tuning, 50 Hz/voltage limit, eight sinusoidal road-case pairs, and reported experimental summaries.
- **Unavailable for exact replication:** complete identified test-rig/valve constants, original fixed-waveform samples, and raw measured body/road/pressure/voltage histories.
- **Deliberate simplifications:** equivalent first-order pressure state, zero cubic spring/tire damping in the default model, full simulated state feedback, symmetric PPF multipliers, and fixed-gain road sweep.
- **Comparator caveat:** BSC is derived for this equivalent actuator and PID has conditional integration under saturation; neither is asserted to be bit-for-bit identical to the paper's real-time comparator code.
- **Theory caveat:** numerical error-ratio clipping and valve saturation can break the assumptions behind the ideal unsaturated PPF guarantee. Check the post-saturation trajectory, not only the requested control law.

## Next work for a teammate

1. **Lock a reference dataset.** Obtain the road command and measured time histories from the authors or digitize the published traces with a documented method; keep fixed-waveform and sinusoidal protocols separate.
2. **Identify the physical plant.** Record mass, suspension/tire force curves, cylinder area, supply pressure, leakage, valve-flow coefficients, pressure scaling, and uncertainty ranges. Replace each assumption only with a cited or measured value.
3. **Rebuild the experimental signal path.** Add the paper's force observer, acceleration-to-velocity integration, sensor sampling/filtering, and any measured delay/noise before comparing hardware-like outputs.
4. **Match tuning and comparators.** Apply the source paper's case-specific gains and reproduce BSC/PID implementation details; document every deviation.
5. **Rerun validation.** Keep MATLAB/Simulink parity, align inputs and sample times, then compare body displacement, acceleration, voltage, IAE/ITAE/ITSE, and explicit PPF violations. Update the report/slides only after this evidence is checked.

Suggested acceptance condition for a future **quantitative** replication: publish the identified parameter set and road trace, provide scripts that regenerate every figure, and report numerical agreement/disagreement with tolerances instead of judging resemblance by eye.

## Repository map

| Path | Responsibility |
| --- | --- |
| [`quarter_car_replication_config.m`](quarter_car_replication_config.m) | Parameters and provenance-sensitive settings |
| [`run_quarter_car_replication.m`](run_quarter_car_replication.m) | MATLAB dynamics, AFC/BSC/PID/passive cases, sweep, metrics |
| [`build_quarter_car_simulink.m`](build_quarter_car_simulink.m) | Editable AFC Simulink model generator |
| [`QuarterCar_AFC_Replication.slx`](QuarterCar_AFC_Replication.slx) | Committed executable Simulink model |
| [`generate_research_assets.m`](generate_research_assets.m) | PDF/PNG figures, model capture, implementation cross-check |
| [`quarter_car_replication_results.mat`](quarter_car_replication_results.mat) | Committed default-run numerical results |
| [`paper_assets/`](paper_assets/) | Vector PDFs, PNG previews, Simulink/MATLAB captures, cross-check data |
| [`QuarterCar_AFC_Research_Report.tex`](QuarterCar_AFC_Research_Report.tex) | Report source (original TikZ schematic and MATLAB figures) |
| [`QuarterCar_AFC_Presentation.tex`](QuarterCar_AFC_Presentation.tex) | Slide source |
| [`deliverables/`](deliverables/) | Final report and presentation PDFs |
| [`Presentation_Notes.md`](Presentation_Notes.md) | Slide-by-slide speaking guide and likely questions |

When taking over, branch from `main`, rerun the baseline before changing parameters, and review generated-file diffs before committing. Do not commit the locally held restricted publisher PDF or present surrogate-model output as measured experimental validation.
