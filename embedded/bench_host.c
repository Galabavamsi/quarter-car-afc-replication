/* Host benchmark: ns per call of each law, plus a numerical self-check
 * against values produced by the MATLAB/Octave library. */
#include <stdio.h>
#include <time.h>
#include "qc_laws.h"
static double now(void){struct timespec ts; clock_gettime(CLOCK_MONOTONIC,&ts); return ts.tv_sec+1e-9*ts.tv_nsec;}
int main(void)
{
    afc_t a = {1, 1, 1e-8, {0.2, 110, 100}, {0.018, 90, 80}, {3, 2, 2}, {25.5, 12, 216}};
    plant_t p = {290, 59, 16812, 0, 1000, 190000, 3.35e-4, 4.515e13, 1, 1.545e9, 10342500, 1e-6, 5e-4};
    bsc_t b = {{400, 100, 400}};
    pidc_t q = {3080, 200, 200, 0, 0.02, 5};
    real x[6] = {0.005, 0.02, 0.01, -0.03, 0.3, 1e-4};
    printf("self-check: afc %.10g  bsc %.10g  pid %.10g\n", (double)afc_law(&a, 0.5, x), (double)bsc_law(&b, &p, x, 0.02), (double)pid_law(&q, x));
    const int N = 2000000; volatile real sink = 0; double t0, dt[3];
    t0 = now(); for (int i = 0; i < N; ++i) { x[0] = (real)(1e-9 * (i & 1023)); sink += afc_law(&a, (real)(i * 1e-6), x); } dt[0] = now() - t0;
    t0 = now(); for (int i = 0; i < N; ++i) { x[0] = (real)(1e-9 * (i & 1023)); sink += bsc_law(&b, &p, x, 0.01); } dt[1] = now() - t0;
    q.I = 0; t0 = now(); for (int i = 0; i < N; ++i) { x[0] = (real)(1e-9 * (i & 1023)); sink += pid_law(&q, x); } dt[2] = now() - t0;
    printf("host ns/call (%s): AFC %.1f  BSC %.1f  PID %.1f\n", sizeof(real) == 4 ? "float" : "double", 1e9 * dt[0] / N, 1e9 * dt[1] / N, 1e9 * dt[2] / N);
    return (int)(sink * 0);
}
