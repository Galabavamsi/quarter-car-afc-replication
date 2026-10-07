function U = qc_kronecker(n, d)
%QC_KRONECKER Deterministic low-discrepancy samples in [0,1)^d.
%   U(i,j) = frac(0.5 + i * alpha_j) with alpha_j = frac(sqrt(prime_j)).
%   Identical in MATLAB and Octave (no RNG state involved), so Monte Carlo
%   tables are reproducible across both.
p = primes(200);
alpha = mod(sqrt(p(1:d)), 1);
U = mod(0.5 + (1:n)' * alpha, 1);
end
