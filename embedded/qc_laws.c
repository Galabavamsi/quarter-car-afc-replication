#include <math.h>
#include "qc_laws.h"

#define EXP(v)  ((sizeof(real) == sizeof(float)) ? (real)expf((float)(v)) : (real)exp((double)(v)))
#define LOG(v)  ((sizeof(real) == sizeof(float)) ? (real)logf((float)(v)) : (real)log((double)(v)))
#define SQRT(v) ((sizeof(real) == sizeof(float)) ? (real)sqrtf((float)(v)) : (real)sqrt((double)(v)))

/* AFC, eqs. (9)-(20): three exponentials, three logarithms, three divisions */
real afc_law(const afc_t *c, real t, const real x[6])
{
    const real src[3] = {x[0], x[1], x[4]};
    real prev = 0, v = 0;
    for (int i = 0; i < 3; ++i) {
        real mu = (c->mu0[i] - c->muinf[i]) * EXP(-c->alpha[i] * t) + c->muinf[i];
        real z = (src[i] - prev) / mu;
        if (z < -c->delta + c->guard) z = -c->delta + c->guard;
        if (z > c->dbar - c->guard) z = c->dbar - c->guard;
        real eps = (real)0.5 * LOG((c->delta + z) / (c->dbar - z));
        v = -c->k[i] * eps;
        prev = v;
    }
    return v;
}

/* model-based backstepping for the valve model (afc2/qc_bsc.m) */
real bsc_law(const bsc_t *c, const plant_t *p, const real x[6], real zr)
{
    real d = x[0] - x[2], v = x[1] - x[3];
    real Fs = p->ks * d + p->ksn * d * d * d, Fd = p->bs * v;
    real F = p->A * x[4] / p->kappa;
    real Ft = p->kt * (x[2] - zr);
    real a2 = (-Fd - Fs + F) / p->ms;               /* body acceleration */
    real a4 = (Fd + Fs - Ft - F) / p->mu;           /* wheel acceleration */
    real Fdd = p->bs * (a2 - a4);
    real Fsd = (p->ks + 3 * p->ksn * d * d) * v;
    real nu1 = x[0], al1 = -c->k[0] * nu1, al1d = -c->k[0] * x[1], al1dd = -c->k[0] * a2;
    real nu2 = x[1] - al1, nu2d = a2 - al1d;
    real al2 = p->kappa * (-c->k[1] * nu2 * p->ms + Fd + Fs + p->ms * al1d - p->ms * nu1) / p->A;
    real al2d = p->kappa * (-c->k[1] * nu2d * p->ms + Fdd + Fsd + p->ms * al1dd - p->ms * x[1]) / p->A;
    real nu3 = x[4] - al2;
    real num = -c->k[2] * nu3 - p->A * nu2 / (p->ms * p->kappa) + p->beta * x[4]
             + p->kappa * p->alpha * p->A * v + al2d;
    real s = (num >= 0) ? 1 : -1;
    real dP = p->Ps - s * x[4] / p->kappa;
    if (dP < 1) dP = 1;
    return num / (p->kappa * p->gamma * p->Kv * SQRT(dP));
}

/* PID with conditional integration */
real pid_law(pidc_t *c, const real x[6])
{
    real e = -x[0];
    real Ic = c->I + c->Ts * e;
    real uc = c->kp * e + c->ki * Ic - c->kd * x[1];
    real au = uc < 0 ? -uc : uc;
    if (au <= c->Vmax || ((uc > 0) != (e > 0))) c->I = Ic;
    return c->kp * e + c->ki * c->I - c->kd * x[1];
}

void afc_fast_init(afc_fast_t *s, const afc_t *c, real Ts)
{
    s->c = *c;
    for (int i = 0; i < 3; ++i) { s->mu[i] = c->mu0[i]; s->decay[i] = EXP(-c->alpha[i] * Ts); }
}

real afc_fast_step(afc_fast_t *s, const real x[6])
{
    const real src[3] = {x[0], x[1], x[4]};
    real prev = 0, v = 0;
    for (int i = 0; i < 3; ++i) {
        real z = (src[i] - prev) / s->mu[i];
        if (z < -s->c.delta + s->c.guard) z = -s->c.delta + s->c.guard;
        if (z > s->c.dbar - s->c.guard) z = s->c.dbar - s->c.guard;
        v = -s->c.k[i] * (real)0.5 * LOG((s->c.delta + z) / (s->c.dbar - z));
        prev = v;
        s->mu[i] = s->c.muinf[i] + (s->mu[i] - s->c.muinf[i]) * s->decay[i];
    }
    return v;
}
