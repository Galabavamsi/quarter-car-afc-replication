function G = qc_sine_gain(P, C, freqs, amp)
%QC_SINE_GAIN Empirical frequency response of the full nonlinear, sampled
%   closed loop: drive a sine road of amplitude amp (m) at each frequency,
%   discard the transient, and least-squares fit the fundamental of z_s and
%   of the body acceleration. Returns gains |z_s|/|z_r| and |a_s|/|z_r|
%   (1/s^2), plus the RMS of the residual (energy NOT at the drive
%   frequency, a measure of limit cycles / harmonics).
if nargin < 4
    amp = 0.005;
end
G.f = freqs(:)';
G.zs = nan(size(G.f));
G.acc = nan(size(G.f));
G.resid_zs = nan(size(G.f));
G.u_rms = nan(size(G.f));
for i = 1:numel(G.f)
    f = G.f(i);
    T = max(8, 12 / f);           % at least 12 periods, at least 8 s
    T = P.sim.Ts * ceil(T / P.sim.Ts);
    R = qc_road('sine', amp, f);
    Ci = C;
    run = qc_simulate(P, R, Ci, struct('T', T));
    if run.diverged
        continue;
    end
    keep = run.t >= T / 2;
    t = run.t(keep);
    w = 2 * pi * f;
    X = [sin(w * t), cos(w * t), ones(size(t))];
    cz = X \ run.x(1, keep)';
    ca = X \ run.acc(keep);
    G.zs(i) = hypot(cz(1), cz(2)) / amp;
    G.acc(i) = hypot(ca(1), ca(2)) / amp;
    G.resid_zs(i) = sqrt(mean((run.x(1, keep)' - X * cz) .^ 2));
    G.u_rms(i) = sqrt(mean(run.u(keep) .^ 2));
end
end
