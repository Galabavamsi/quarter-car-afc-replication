function M = qc_metrics(run, P)
%QC_METRICS Performance indices used by Na et al. plus extra diagnostics.
%   Paper (eqs. 24-26): IAE, ITAE, ITSE on z_s over 0-T.
%   Paper (Fig. 8/13): acceleration RMS/max, control RMS/max.
%   Paper (Fig. 9):    acceleration PSD energy in 0-4, 4-8, 8-20 Hz bands.
%   Extra: saturation fraction, prescribed-bound violation (max and
%   fraction of time), suspension travel, dynamic tire-load ratio.
M.diverged = run.diverged;
ok = all(isfinite(run.x), 1)';
t = run.t(ok);
zs = run.x(1, ok)';
zu = run.x(3, ok)';
acc = run.acc(ok);
u = run.u(ok);
uraw = run.u_raw(ok);
mu1 = run.mu1(ok);
zr = run.zr(ok);
if run.diverged || numel(t) < 2
    names = {'IAE', 'ITAE', 'ITSE', 'acc_rms', 'acc_max', 'u_rms', 'u_max', ...
        'sat_frac', 'ppf_viol', 'ppf_viol_frac', 'travel_max', 'tire_ratio_max', ...
        'band_0_4', 'band_4_8', 'band_8_20'};
    for i = 1:numel(names)
        M.(names{i}) = NaN;
    end
    return;
end
M.IAE = trapz(t, abs(zs));
M.ITAE = trapz(t, t .* abs(zs));
M.ITSE = trapz(t, t .* zs .^ 2);
M.acc_rms = sqrt(mean(acc .^ 2));
M.acc_max = max(abs(acc));
M.u_rms = sqrt(mean(u .^ 2));
M.u_max = max(abs(u));
M.sat_frac = mean(abs(uraw) > run.Vmax);
M.ppf_viol = max([0; abs(zs) - mu1]);
M.ppf_viol_frac = mean(abs(zs) > mu1);
M.travel_max = max(abs(zs - zu));
staticLoad = (P.mech.ms + P.mech.mu) * P.g;
M.tire_ratio_max = max(abs(P.mech.kt * (zu - zr))) / staticLoad;
[f, Pxx] = qc_psd(acc, 1 / P.sim.Ts);
M.band_0_4 = band(f, Pxx, 0, 4);
M.band_4_8 = band(f, Pxx, 4, 8);
M.band_8_20 = band(f, Pxx, 8, 20);
end

function e = band(f, Pxx, f1, f2)
% mean-square acceleration contained in [f1, f2) Hz, (m/s^2)^2
sel = f >= f1 & f < f2;
if nnz(sel) < 2
    e = 0;
else
    e = trapz(f(sel), Pxx(sel));
end
end
