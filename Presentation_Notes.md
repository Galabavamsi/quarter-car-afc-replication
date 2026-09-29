# Presentation notes (about 10-12 minutes)

These notes follow `QuarterCar_AFC_Presentation.pdf`. Keep one distinction explicit throughout: **paper-reported experiment** versus **our representative-parameter simulation**.

1. **Title.** Introduce Na et al.'s electro-hydraulic quarter-car AFC paper. State that this is a control-method reproduction, not a claim to have rebuilt their physical test rig.
2. **Research question.** Explain the tension between ride comfort, a shrinking displacement envelope, and a finite valve command. Ask what is verifiable when actuator identification data are missing.
3. **Physical system.** Point out the sprung and unsprung masses, suspension spring/damper, tire, road displacement, and equal-and-opposite actuator force. Trace the force signs in the two mass equations.
4. **Prescribed performance.** The envelope starts wide, contracts exponentially, and settles at a specified residual tolerance. The logarithmic transformation goes singular at its boundaries, making the constraint visible to the feedback law.
5. **AFC recursion.** Walk through `e1`, `e2`, and `e3`: body displacement, velocity relative to virtual `u1`, then pressure state relative to virtual `u2`. Emphasize that this is *approximation-free control*, not feedforward control.
6. **Paper evidence.** Quote the paper's 50 Hz implementation, +/-5 V limit, fixed-waveform acceleration statistics, and computation times. Warn that these numbers belong to a different excitation from our Case 10 run.
7. **Simulink controller row.** Identify the AFC MATLAB Function block, controller time/state feedback tags, 50 Hz zero-order hold, and control log. This is the top row of the runnable `.slx` model.
8. **Simulink plant row.** Identify the road source, quarter-car plant, state integrator, and logged road/state/acceleration outputs. Tagged connections are a layout choice, not a change in dynamics.
9. **Parameter ledger.** Distinguish paper-provided controller/road/sample settings from representative mechanical constants and assumed equivalent actuator coefficients. The real rig's valve coefficients are not given in the source.
10. **Numerical protocol.** The main road is 43 mm amplitude at 0.505 Hz for 20 seconds. MATLAB uses four RK4 substeps each 20 ms sample; Simulink uses 1 ms fixed-step `ode4`.
11. **Response capture.** Point to road and body traces. All three active controllers beat passive displacement IAE here, but PID narrowly beats AFC under our assumed plant.
12. **PPF audit.** In the bottom panel, any ratio over 1 is a violation. The maximum absolute displacement bound exceedance is 18.85 mm. Do not imply the hardware paper reported this violation.
13. **Actuator limit.** The applied AFC valve voltage spends much of the run at +/-5 V. The *raw* command would exceed the rail in 85.4% of samples. Saturation is a plausible reason the ideal envelope guarantee does not transfer to this implementation.
14. **Controller ranking.** Read the actual IAE values: AFC 0.4849, PID 0.4698, passive 0.6709 m s. The assumed model does not reproduce the paper's experimental controller ranking.
15. **Road sweep.** The eight input pairs come from the source paper, but our gains stay fixed. The non-monotonic error indices therefore describe sensitivity of this surrogate model, not the paper's measurements.
16. **Cross-check.** The independently executed MATLAB and Simulink AFC body traces differ by at most about 2.10e-8 m. That strongly checks implementation consistency, but says nothing about identification accuracy.
17. **Conclusion.** We reproduced the method and exposed its dependencies. For a quantitative comparison, obtain rig/valve constants and original waveforms, implement the observer/sensor path, and retune cases exactly as described in the paper.
18. **References.** Keep the source paper and two separate parameter-source studies available for questions.

## Likely questions

- **Why is AFC not best in your simulation?** The paper's physical rig, nonlinear valve, observer path, input protocol, and case tuning are not fully reconstructed. Our assumed actuator frequently saturates. The result is a model-dependent diagnostic, not evidence that the experimental claim is false.
- **Does the PPF theorem fail?** The published design addresses an ideal control law under its assumptions. Here the applied command is voltage-clipped, and the code guards the logarithm numerically; neither operation guarantees the original proof conditions.
- **Why use these masses and stiffnesses?** They are cited representative values from a different quarter-car study and are configurable. They are not claimed as measurements from Na et al.'s test rig.
- **What does the Simulink check validate?** Agreement between two numerical implementations of the *same assumed model*, not agreement with hardware.
