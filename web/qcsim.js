// qcsim.js - quarter-car + servo-valve + AFC simulator (mirrors afc2/*.m)
const QC = (() => {
  const PLANTS = {
    cal: { label: 'Calibrated hydraulic (Alleyne–Hedrick, τ = 33 ms)', model: 'valve',
      ms: 290, mu: 59, ks: 16812, bs: 1000, kt: 190000, A: 3.35e-4, alpha: 4.515e13, beta: 1,
      gamma: 1.545e9, Ps: 10342500, kappa: 1e-6, Kv: 5e-4, tau: 1 / 30, h: 0.001 },
    lit: { label: 'Literature spool lag only (τ = 3 ms)', model: 'valve',
      ms: 290, mu: 59, ks: 16812, bs: 1000, kt: 190000, A: 3.35e-4, alpha: 4.515e13, beta: 1,
      gamma: 1.545e9, Ps: 10342500, kappa: 1e-6, Kv: 5e-4, tau: 0.003, h: 0.001 },
    v1: { label: 'v1 surrogate actuator (first-order pressure)', model: 'equivalent',
      ms: 320, mu: 40, ks: 18000, bs: 1000, kt: 200000, A: 3.35e-4, kappa: 1e-6, beta: 1,
      cRel: 0.05, bu: 0.75, Plim: 10.3425e6, h: 0.005 },
  };
  const ROADS = { 3: [0.025, 0.188], 4: [0.043, 0.204], 5: [0.043, 0.243], 6: [0.043, 0.307],
    7: [0.043, 0.361], 8: [0.043, 0.407], 9: [0.043, 0.423], 10: [0.043, 0.505] };
  const AFC = {
    A: { mu0: [0.2, 110, 100], muinf: [0.018, 90, 80], alpha: [3, 2, 2], k: [25.5, 12, 216] },
    B: { mu0: [0.12, 80, 50], muinf: [0.018, 40, 20], alpha: [2, 2, 4], k: [12, 5, 146] },
  };

  const isoCache = new Map();
  function road(spec) {
    if (spec.kind === 'case') {
      const [A0, f] = ROADS[spec.peaks]; const A = A0 * (spec.scale ?? 1); const w = 2 * Math.PI * f;
      return t => [A * Math.sin(w * t), A * w * Math.cos(w * t)];
    }
    if (spec.kind === 'bump') {
      const H = 0.05 * (spec.scale ?? 1), L = 2.5, v = 20 / 3.6, t0 = 1.0, d = L / v;
      return t => (t >= t0 && t <= t0 + d)
        ? [H / 2 * (1 - Math.cos(2 * Math.PI * (t - t0) / d)), H / 2 * (2 * Math.PI / d) * Math.sin(2 * Math.PI * (t - t0) / d)]
        : [0, 0];
    }
    // ISO 8608 class C, 54 km/h, 200 sinusoids, deterministic phases (same as qc_road.m, seed 1)
    const ckey = String(spec.scale ?? 1) + '|' + (spec.T ?? 20);
    if (isoCache.has(ckey)) return isoCache.get(ckey);
    const Nn = 200, n0 = 0.1, Gd0 = 256e-6 * (spec.scale ?? 1) ** 2, v = 15;
    const amp = [], w = [], phi = [];
    const dn = (2.83 - 0.011) / (Nn - 1);
    for (let i = 0; i < Nn; i++) {
      const n = 0.011 + i * dn; const Gd = Gd0 * (n / n0) ** -2;
      amp.push(Math.sqrt(2 * Gd * dn)); w.push(2 * Math.PI * n * v);
      phi.push(2 * Math.PI * (((i + 1) * 0.6180339887498949 + 0.7548776662466927) % 1));
    }
    const exact = t => { let z = 0, zd = 0; for (let i = 0; i < Nn; i++) { const a = w[i] * t + phi[i]; z += amp[i] * Math.sin(a); zd += amp[i] * w[i] * Math.cos(a); } return [z, zd]; };
    // tabulate on a 0.5 ms grid: every RK4 stage time of a 1 ms or 5 ms step lands on it exactly
    const dg = 0.0005, M = Math.round((spec.T ?? 20) / dg) + 2, tab = new Float64Array(M);
    for (let i = 0; i < M; i++) tab[i] = exact(i * dg)[0];
    const fn = t => { const i = Math.round(t / dg); return (i < M && Math.abs(t - i * dg) < 1e-9) ? [tab[i], 0] : exact(t); };
    isoCache.set(ckey, fn);
    return fn;
  }

  function rhs(p, x, u, zr, passive, out) {
    const d = x[0] - x[2], v = x[1] - x[3];
    const Fs = p.ks * d, Fd = p.bs * v, Ft = p.kt * (x[2] - zr);
    let F = 0; out[4] = 0; out[5] = 0;
    if (!passive) {
      F = p.A * x[4] / p.kappa;
      if (p.model === 'equivalent') {
        let d5 = -p.beta * x[4] - p.cRel * v + p.bu * u; const lim = p.kappa * p.Plim;
        if ((x[4] >= lim && d5 > 0) || (x[4] <= -lim && d5 < 0)) d5 = 0;
        out[4] = d5;
      } else {
        const PL = x[4] / p.kappa; let xv;
        if (p.tau > 0) { xv = x[5]; out[5] = (p.Kv * u - x[5]) / p.tau; } else xv = p.Kv * u;
        const dP = p.Ps - Math.sign(xv) * PL;
        out[4] = p.kappa * (-p.beta * PL - p.alpha * p.A * v + p.gamma * xv * Math.sqrt(Math.max(dP, 0)));
      }
    }
    out[0] = x[1]; out[1] = (-Fd - Fs + F) / p.ms; out[2] = x[3]; out[3] = (Fd + Fs - Ft - F) / p.mu;
    return out;
  }

  function afcLaw(c, t, x, rho) {
    const src = [x[0], x[1], x[4]]; let prev = 0, v = 0; const zeta = [0, 0, 0];
    for (let i = 0; i < 3; i++) {
      const mu = ((c.mu0[i] - c.muinf[i]) * Math.exp(-c.alpha[i] * t) + c.muinf[i]) * (1 + rho);
      zeta[i] = (src[i] - prev) / mu;
      const z = Math.min(Math.max(zeta[i], -1 + 1e-8), 1 - 1e-8);
      v = -c.k[i] * 0.5 * Math.log((1 + z) / (1 - z)); prev = v;
    }
    return [v, zeta];
  }

  // cfg: {plant, ctrl:{type, variant, kscale, mu1inf, alpha1}, road:{...}, T, Kscale, tau}
  function simulate(cfg) {
    const p = Object.assign({}, PLANTS[cfg.plant]);
    if (p.model === 'valve') { p.Kv *= cfg.valveScale ?? 1; if (cfg.tauMs != null) p.tau = cfg.tauMs / 1000; }
    else { p.bu *= cfg.valveScale ?? 1; }
    const Ts = 0.02, T = cfg.T ?? 20, N = Math.round(T / Ts), sub = Math.max(1, Math.round(Ts / p.h)), h = Ts / sub;
    const R = road(cfg.road), ct = cfg.ctrl.type, passive = ct === 'passive';
    const base = AFC[cfg.ctrl.variant ?? 'A'];
    const c = { mu0: base.mu0.slice(), muinf: base.muinf.slice(), alpha: base.alpha.slice(), k: base.k.map(k => k * (cfg.ctrl.kscale ?? 1)) };
    if (cfg.ctrl.mu1inf != null) { c.muinf[0] = cfg.ctrl.mu1inf; c.mu0[0] = Math.max(c.mu0[0], c.muinf[0] * 1.5); }
    if (cfg.ctrl.alpha1 != null) c.alpha[0] = cfg.ctrl.alpha1;
    const x = new Float64Array(6), k1 = new Float64Array(6), k2 = new Float64Array(6), k3 = new Float64Array(6), k4 = new Float64Array(6), tmp = new Float64Array(6);
    const out = { t: new Float64Array(N + 1), zs: new Float64Array(N + 1), zu: new Float64Array(N + 1), zr: new Float64Array(N + 1),
      u: new Float64Array(N + 1), uraw: new Float64Array(N + 1), acc: new Float64Array(N + 1), mu1: new Float64Array(N + 1),
      zeta1: new Float64Array(N + 1), diverged: false, c, p };
    let I = 0, rho = 0; const Vmax = 5;
    const kp = 3080, ki = 200, kd = 200;
    for (let k = 0; k <= N; k++) {
      const t = k * Ts; out.t[k] = t;
      const [zrk] = R(t); out.zr[k] = zrk; out.zs[k] = x[0]; out.zu[k] = x[2];
      out.mu1[k] = (c.mu0[0] - c.muinf[0]) * Math.exp(-c.alpha[0] * t) + c.muinf[0];
      let ur = 0;
      if (ct === 'afc' || ct === 'afc_aw') {
        const [v] = afcLaw(c, t, x, ct === 'afc_aw' ? rho : 0); ur = v;
        if (ct === 'afc_aw') rho = Math.max(0, rho + Ts * (-0.5 * rho + 5 * Math.max(Math.abs(ur) - Vmax, 0) / Vmax));
      } else if (ct === 'skyhook') {
        ur = -2.7 * (x[4] - p.kappa * (-4000 * x[1] - 1e5 * x[0]) / p.A);
      } else if (ct === 'pid') {
        const e = -x[0], Ic = I + Ts * e, uc = kp * e + ki * Ic - kd * x[1];
        if (Math.abs(uc) <= Vmax || Math.sign(uc) !== Math.sign(e)) I = Ic;
        ur = kp * e + ki * I - kd * x[1];
      }
      if (!isFinite(ur)) ur = Math.sign(ur) * 1e6;
      const u = Math.min(Math.max(ur, -Vmax), Vmax); out.uraw[k] = ur; out.u[k] = u;
      out.acc[k] = rhs(p, x, u, zrk, passive, tmp)[1];
      out.zeta1[k] = x[0] / out.mu1[k];   // audited against the nominal envelope
      if (k === N) break;
      for (let j = 0; j < sub; j++) {
        const tj = t + j * h;
        rhs(p, x, u, R(tj)[0], passive, k1);
        for (let i = 0; i < 6; i++) tmp[i] = x[i] + h / 2 * k1[i];
        rhs(p, tmp, u, R(tj + h / 2)[0], passive, k2);
        for (let i = 0; i < 6; i++) tmp[i] = x[i] + h / 2 * k2[i];
        rhs(p, tmp, u, R(tj + h / 2)[0], passive, k3);
        for (let i = 0; i < 6; i++) tmp[i] = x[i] + h * k3[i];
        rhs(p, tmp, u, R(tj + h)[0], passive, k4);
        for (let i = 0; i < 6; i++) x[i] += h / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
      }
      if (!x.every(Number.isFinite) || Math.abs(x[0]) > 10) { out.diverged = true; out.N = k; break; }
    }
    out.N = out.N ?? N;
    out.metrics = metrics(out);
    return out;
  }

  function trapz(t, y, n) { let s = 0; for (let i = 1; i <= n; i++) s += 0.5 * (t[i] - t[i - 1]) * (y(i) + y(i - 1)); return s; }
  function metrics(o) {
    const n = o.N, t = o.t;
    const IAE = trapz(t, i => Math.abs(o.zs[i]), n), ITAE = trapz(t, i => t[i] * Math.abs(o.zs[i]), n), ITSE = trapz(t, i => t[i] * o.zs[i] ** 2, n);
    let a2 = 0, u2 = 0, sat = 0, viol = 0, zmax = 0, travel = 0;
    for (let i = 0; i <= n; i++) {
      a2 += o.acc[i] ** 2; u2 += o.u[i] ** 2; if (Math.abs(o.uraw[i]) > 5) sat++;
      viol = Math.max(viol, Math.abs(o.zs[i]) - o.mu1[i]); zmax = Math.max(zmax, Math.abs(o.zs[i]));
      travel = Math.max(travel, Math.abs(o.zs[i] - o.zu[i]));
    }
    return { IAE, ITAE, ITSE, accRMS: Math.sqrt(a2 / (n + 1)), uRMS: Math.sqrt(u2 / (n + 1)), sat: sat / (n + 1),
      viol: Math.max(0, viol), zmax, travel };
  }

  function linearAFC(cfg) {
    const p = PLANTS[cfg.plant]; const c = AFC[cfg.ctrl.variant ?? 'A']; const ks = cfg.ctrl.kscale ?? 1;
    const m1 = cfg.ctrl.mu1inf ?? c.muinf[0];
    const g1 = ks * c.k[0] / m1, g2 = ks * c.k[1] / c.muinf[1], g3 = ks * c.k[2] / c.muinf[2];
    return { g1, g2, g3, spring: p.A / p.kappa * g2 * g1, damper: p.A / p.kappa * g2, Kx1: -g3 * g2 * g1, Kx2: -g3 * g2, Kx5: -g3 };
  }
  return { simulate, linearAFC, PLANTS };
})();
if (typeof module !== 'undefined') module.exports = QC;
