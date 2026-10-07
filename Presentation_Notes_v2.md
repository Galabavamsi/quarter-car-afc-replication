# Speaker notes for the v2 presentation (about 15 minutes)

Keep one distinction explicit throughout: numbers **printed in the paper**, numbers **read off its plots**, values **calibrated** or **assumed** by us, and our **simulation** outputs.

1. **Title.** The left image is our FreeCAD model of a test rig like the paper's Fig. 1. The right image is a Blender frame driven by our simulated trajectories.
2. **Questions.** Four questions: method, reproduction, validity, extension. We built five implementations of the same equations and checked each against the others, so the results don't hinge on one piece of code.
3. **Plant.** Point to the servo-valve law. The paper never publishes the pressure scale κ, the valve gain, or any valve lag, and these three numbers end up deciding everything.
4. **AFC.** Walk through the envelope, the log transform and the three proportional steps. The proof turns bounded ε into bounded ζ, but it assumes continuous time and an unsaturated input, and it promises boundedness, not convergence.
5. **v1 diagnosis.** Our first model saturated badly, and the v1 report blamed actuator authority. Linearising shows the loop is unstable with these gains, and removing the rail makes it worse. The gains are numbers in the rig's units.
6. **Calibration.** We take a standard hydraulic quarter car from the literature and fit two scale factors on a single paper experiment. The paper's unchanged gains then give IAE 0.129 against 0.122, ITAE 1.20 against 1.14, ITSE 0.0088 against 0.0093, and a 0.5 V command. BSC and PID saturate, as the paper reports.
7. **Case 10 traces.** Compare with paper Figs. 6 and 10. AFC stays inside the envelope with a smooth command. The comparators switch rail to rail on our plant.
8. **Cross-checks.** Before trusting any number, we rebuilt the plant three more ways. Simulink with the same sensor noise matches to rounding error. Simscape Multibody assembles the mechanics from joints instead of our equations and still matches to 2 micrometres. Its only discrepancy was a start-up offset: Simscape began with the wheel moving with the road. MATLAB and Octave agree everywhere except a few saturated runs, where the unstable loop on the rail amplifies rounding.
9. **Hold-out.** This is the honest part. The paper's Case-B gains don't carry over. A joint fit to two gain sets fails on the held-out roads, so the paper can't be reproduced to the digit. We compare trends.
10. **Lemma 1.** On the calibrated plant, max |ζ₁| = 0.50 and Lemma 2's tanh bound is 0.504, so the bound is tight. On v1 every bound crossing happens while the valve is saturated.
11. **Skyhook view.** Near zero error the law is a 63 kN/m skyhook spring with almost no damping. That's why it suppresses displacement well, and also why it does nothing for the 4–8 Hz comfort band.
12. **Sampled data.** The continuous loop is stable, but every zero-order hold we tried is unstable, because an 88 Hz hydraulic mode loses its damping. The nonlinear system settles into a bounded oscillation. That is consistent with the Lemma, but a continuous-time proof doesn't cover the digital controller.
13. **Fair comparators.** At the same voltage, skyhook and LQR do better. Halving the AFC gains makes things worse, because the error reaches the barrier zone where the effective gain blows up.
14. **Robustness and transients.** Monte Carlo: AFC usually keeps the bound, but its failures are abrupt. On a bump the paper's AFC saturates and the wheel leaves the road.
15. **Extension.** One extra state lets the envelope widen while the valve is saturated. The bump is fixed in both the quarter car and the half car, and nothing changes on Case 10.
16. **Realism.** The paper's sensor choice (integrating the accelerometer) is right. On a Cortex-M4 without FPU the AFC law costs twice the instructions of BSC, so the timing advantage in Table III must come from elsewhere.
17. **Verdicts.** Read the table. Most claims are reproduced or qualified. Only the computation-time claim isn't supported.
18. **Tools.** The CAD rig, Blender animation and web lab exist for explanation, not for evidence.
19. **Conclusions.** The method is sound when the actuator can keep up. The three missing rig numbers are what stand between this study and a quantitative replication.

## Likely questions

- **Isn't the calibration overfitting?** It fits two factors to four numbers from one experiment. That is why we ran a hold-out test, and the hold-out fails for Case B, which we report.
- **Why is the lumped lag 33 ms when servo valves respond in 3 ms?** It's an effective lag covering the valve, the ESO estimate and filtering. With 3 ms the fit error is 62–69% (the loop is strongly unstable there, so the exact value depends on the platform). The paper itself mentions actuator delays.
- **Does local instability contradict the paper?** No. Lemma 1 claims boundedness inside the envelope, which still holds; the oscillation is 0.13 mm. It does show that the continuous-time proof doesn't analyse the 50 Hz implementation.
- **Is the anti-windup AFC still prescribed-performance control?** It deliberately gives the bound up while the actuator can't deliver, and recovers it afterwards at rate 0.5/s. It's a graceful-degradation design, not a guarantee.
- **Why trust our numbers?** Five implementations agree with each other: Simulink to 1e-16 m, Simscape Multibody to 2 µm, the C law to 10 digits. The v1 numbers are reproduced exactly.
