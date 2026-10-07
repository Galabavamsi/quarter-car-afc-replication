/* qc_laws.h - the three control laws of Na et al. (2022) in portable C.
 * REAL is float or double (compile with -DQC_REAL=float for single precision).
 * These mirror afc2/qc_afc.m, afc2/qc_bsc.m (valve model) and the PID in
 * afc2/qc_controller.m line for line, so timing compares like with like. */
#ifndef QC_LAWS_H
#define QC_LAWS_H
#ifndef QC_REAL
#define QC_REAL double
#endif
typedef QC_REAL real;

typedef struct { real delta, dbar, guard, mu0[3], muinf[3], alpha[3], k[3]; } afc_t;
typedef struct { real ms, mu, ks, ksn, bs, kt, A, alpha, beta, gamma, Ps, kappa, Kv; } plant_t;
typedef struct { real k[3]; } bsc_t;
typedef struct { real kp, ki, kd, I, Ts, Vmax; } pidc_t;

real afc_law(const afc_t *c, real t, const real x[6]);
real bsc_law(const bsc_t *c, const plant_t *p, const real x[6], real zr);
real pid_law(pidc_t *c, const real x[6]);

/* AFC with the envelopes advanced recursively once per sample,
 * mu_{k+1} = mu_inf + (mu_k - mu_inf) * exp(-alpha*Ts), so no exp() at run time.
 * Call afc_fast_init once, then afc_fast_step every sample. */
typedef struct { afc_t c; real mu[3], decay[3]; } afc_fast_t;
void afc_fast_init(afc_fast_t *s, const afc_t *c, real Ts);
real afc_fast_step(afc_fast_t *s, const real x[6]);
#endif
