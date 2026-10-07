# Quarter-car AFC, v2: reproduction, diagnosis, extensions

This is the second stage of the course project on Na et al., *Active Suspension Control of Quarter-Car System With Experimental Validation* (IEEE TSMC 52(8), 2022). The v1 files (`run_quarter_car_replication.m`, `QuarterCar_AFC_Replication.slx`, the v1 report and slides) are unchanged and still reproduce their numbers. v2 adds a modular library, a calibrated literature plant, control-theoretic analysis, extensions, four more independent implementations, and tools for visuals.

**Deliverables:** `deliverables/QuarterCar_AFC_Report_v2.pdf` (10 pages) and `deliverables/QuarterCar_AFC_Presentation_v2.pdf` (19 slides), with speaker notes in `Presentation_Notes_v2.md`.

## Run it (MATLAB, from the repository root)

| Command | What it does | Output |
| --- | --- | --- |
| `run_phase1()` | v1 diagnosis, plant calibration, Case 10 vs paper, 8-road hold-out, feasibility maps, sampled-data stability | `results/phase1/` |
| `run_phase2()` | builds `QuarterCar_AFC_v2.slx` and cross-checks it, paper sensing path (noise, integrator, Kalman), control-law timing | `results/phase2/` |
| `run_phase2b()` | builds `QuarterCar_Multibody.slx` (Simscape Multibody) and cross-checks it | `results/phase2b/` |
| `run_phase3()` | Lyapunov/transformed errors, frequency response, LQR/skyhook trade-off, Monte Carlo, bump + ISO 8608 roads, anti-windup AFC | `results/phase3/` |
| `run_phase3b()` | half car (heave + pitch), decentralised AFC | `results/phase3b/` |
| `export_animation_data()` | trajectories for Blender | `animation/*.csv` |

Every number in the report is printed to a `*_summary.txt` file by these scripts. `results/` is the MATLAB R2026a run that the report uses (whole study about 4 minutes). `results_octave/` holds the same runs made with GNU Octave 8.4, so the study can be checked without MATLAB. The two agree on every unsaturated run. The report and slides take figures from `results/` when it exists and fall back to `results_octave/` otherwise. Rebuild them with `pdflatex` (twice each).

**Cross-checks (MATLAB R2026a):** Simulink v2 vs the library, with the same sensor noise: max |Δz_s| = 4e-16 m and max |Δu| = 2e-12 V. Simscape Multibody (mechanics from joints) vs the library: max |Δz_s| = 2.1 µm, with IAE 0.1294 in both.

**Headless runs:** `matlab -batch "addpath('tools'); qc_agent_runner drain"` executes every `.m` job dropped into `agent_jobs/inbox/` and logs it to `agent_jobs/logs/`. Use this rather than the timer mode (`qc_agent_runner start`) for Simulink jobs. Building a model inside a timer callback in a no-desktop session can hang it.

## Main findings

1. **v1 failure, correctly attributed.** On the v1 plant, the paper's Case-A gains give a locally unstable 50 Hz loop (spectral radius 1.02). Removing the ±5 V rail makes the response worse (IAE 0.48 → 2.37 m s), so saturation was not the cause.
2. **Calibrated reproduction.** This uses the Alleyne–Hedrick hydraulic quarter car with the paper's servo-valve law. Two free factors were fitted on one paper experiment: valve gain 5e-4 m/V and a lumped lag of 33 ms. The paper's *unchanged* AFC then gives IAE 0.129 / ITAE 1.20 / ITSE 0.0088 against the paper's 0.122 / 1.14 / 0.0093, with about 0.5 V RMS command. BSC and PID saturate, as in the paper, and AFC ranks first.
3. **Identifiability.** The paper's Case-B gains fail the hold-out test. A joint fit across both gain sets gets a train error of 0.22 but a test error of 0.63, so the published data cannot pin down the rig.
4. **Theory.** Lemma 1 holds exactly where the valve is unsaturated. Near zero error, AFC is a skyhook spring of 63 kN/m. The continuous loop is stable, but every ZOH implementation is unstable through an 88 Hz hydraulic mode, which leaves a bounded oscillation. At equal effort, LQR and skyhook match or beat AFC.
5. **Transients.** On a 50 mm bump the paper's AFC switches rail to rail and the wheel leaves the road (tyre-load ratio 1.77).
6. **Extension.** A saturation-driven envelope relaxation (`afc_aw`) removes the bump chatter in both the quarter car and the half car, and does not change Case 10.
7. **Timing.** On a Cortex-M4 without FPU, the AFC law takes 18k instructions per call against BSC's 9k. The paper's computation-time advantage does not come from the control law.

## Layout

| Path | Contents |
| --- | --- |
| `afc2/` | the library (see `afc2/README.md`) |
| `build_quarter_car_simulink_v2.m`, `build_quarter_car_multibody.m` | model generators |
| `embedded/` | C versions of the laws, host and Cortex-M4 (QEMU) benchmarks |
| `cad/` | FreeCAD macro of the test rig, STEP/STL export, Blender rig renderer |
| `animation/` | Blender schematic scene, CSV trajectories, rendered videos |
| `web/` | interactive web lab (single HTML file, opens in any browser) |
| `results/`, `results_octave/` | generated tables, figures and summaries |

## Tools

* **FreeCAD:** open `cad/quarter_car_rig.FCMacro` (Macro → Macros… → Execute). It builds the parametric rig and exports `cad/export/*.stl` and `quarter_car_rig.step`.
* **Web:** the project site and interactive lab are live at https://quarter-car-afc.vercel.app (lab at `/lab`). The sources are `web/index.html` and `web/quarter_car_afc_lab.html`; both also work opened locally.
* **Blender:** in the Scripting tab, run `animation/build_quarter_car_scene.py` (schematic, 3 controllers side by side) or `cad/render_rig.py` (CAD rig). Both can also run headless, e.g. `blender --background --python cad/render_rig.py -- --csv ../animation/anim_bump.csv --ctrl afc --out rig_bump_afc.mp4`.
* **Web lab:** open `web/quarter_car_afc_lab.html` in a browser.

## Evidence labels
**Paper** = printed in Na et al.; **Approx.** = read off a paper figure by eye; **Calibrated** = free factor fitted to paper data; **Assumed** = not fixed by any source; **Sim** = this project's output.
