function g = qc_randn(n, seed)
%QC_RANDN Reproducible standard-normal samples (n x 1), identical in MATLAB
%   and Octave: 32-bit linear congruential generator + Box-Muller.
%   Not for cryptography; good enough for sensor-noise studies.
if nargin < 2
    seed = 1;
end
m = 2 ^ 32;
a = 1664525;
c = 1013904223;
s = mod(floor(abs(seed)) * 2654435761 + 12345, m);
k = 2 * ceil(n / 2);
u = zeros(k, 1);
for i = 1:k
    s = mod(a * s + c, m);
    u(i) = (s + 0.5) / m;
end
u1 = u(1:2:end);
u2 = u(2:2:end);
r = sqrt(-2 * log(u1));
g = [r .* cos(2 * pi * u2); r .* sin(2 * pi * u2)];
g = g(1:n);
end
