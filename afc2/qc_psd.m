function [f, Pxx] = qc_psd(y, fs, nfft)
%QC_PSD One-sided Welch PSD (Hann window, 50% overlap, mean removed).
%   Toolbox-free so it runs identically in MATLAB and Octave.
%   Units: (units of y)^2 / Hz. Integral over f equals the variance of y.
if nargin < 3
    nfft = 256;
end
y = y(:) - mean(y(:));
n = numel(y);
nfft = min(nfft, 2 ^ floor(log2(n)));
w = 0.5 - 0.5 * cos(2 * pi * (0:nfft - 1)' / (nfft - 1));
U = sum(w .^ 2);
step = nfft / 2;
starts = 1:step:(n - nfft + 1);
acc = zeros(nfft, 1);
for s = starts
    seg = y(s:s + nfft - 1) .* w;
    acc = acc + abs(fft(seg)) .^ 2;
end
acc = acc / (numel(starts) * fs * U);
Pxx = acc(1:nfft / 2 + 1);
Pxx(2:end - 1) = 2 * Pxx(2:end - 1);
f = (0:nfft / 2)' * fs / nfft;
end
