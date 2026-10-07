# Embedded cost of the three control laws (paper Table III)

The paper reports mean online computation times on a Freescale Kinetis MK60D: **AFC 4.2 ms, BSC 5.8 ms, PID 2.6 ms**. The MK60D is a 100 MHz Cortex-M4 without an FPU. This folder measures the control laws *themselves* with the same arithmetic as `afc2/` (verified to 10 digits against MATLAB/Octave).

* `qc_laws.c/.h`: AFC (eqs. 9-20), model-based BSC (valve model), PID with conditional integration, plus `afc_fast_step`, which advances the envelopes recursively (no `exp` at run time).
* `bench_host.c`: ns/call on the build machine.
* `bench_m4.c`, `m4.ld`: bare-metal Cortex-M4 image for QEMU `mps2-an386`. With `-icount shift=0` the virtual clock ticks once per guest instruction, so SysTick (calibrated against a 2-instruction loop) gives **instruction counts per call**.
* `run_bench.sh`: rebuilds and reruns everything and writes `results_m4.txt`.

Converting instructions to time on a 100 MHz M4 needs a CPI. Soft-float code with flash wait states runs at roughly 1.0-1.5, so the result is a range, not a measurement.

| Law (double, soft-float, like MK60D) | instructions/call | est. time @100 MHz | paper |
| --- | ---: | ---: | ---: |
| AFC | 18 070 | 0.18-0.27 ms | 4.2 ms |
| AFC, recursive envelopes | 11 264 | 0.11-0.17 ms | - |
| BSC | 9 275 | 0.09-0.14 ms | 5.8 ms |
| PID | 940 | 0.01 ms | 2.6 ms |

Reading: the laws alone cost about 15-280x less than the paper's times, so Table III mostly measures something else in the loop (ESO, sensor acquisition, I/O). Taken alone, AFC is **more** expensive than model-based BSC, because it calls 3 `exp` and 3 `log` per sample. Advancing the envelopes recursively removes the `exp` calls and brings AFC close to BSC. The paper's computational advantage for AFC therefore does not come from the control law.
