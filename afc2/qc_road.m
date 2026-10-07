function R = qc_road(kind, varargin)
%QC_ROAD Road excitation profiles. R.fun(t) returns [zr, zrdot].
%
%   R = qc_road('case', peaks)          paper Table IV case (peaks = 3..10)
%   R = qc_road('sine', A, f)           zr = A sin(2 pi f t)
%   R = qc_road('bump', H, L, v, t0)    (1-cos) bump of height H (m), length
%                                       L (m) driven at v (m/s), starting t0
%   R = qc_road('iso8608', cls, v, seed) ISO 8608 random road, class 'A'..'D',
%                                       speed v (m/s), sum of 200 sinusoids
%                                       (deterministic phases from seed)
%
%   All profiles are closed-form in t, so RK4 substeps see the exact road.

switch lower(kind)
    case 'case'
        peaks = varargin{1};
        tab = qc_road_table();
        i = find(tab(:, 1) == peaks, 1);
        if isempty(i)
            error('qc_road:case', 'Paper road cases are 3..10 peaks.');
        end
        R = qc_road('sine', tab(i, 2), tab(i, 3));
        R.name = sprintf('Case %d peaks (%.3f m, %.3f Hz)', peaks, tab(i, 2), tab(i, 3));
        R.peaks = peaks;

    case 'sine'
        A = varargin{1};
        f = varargin{2};
        w = 2 * pi * f;
        R.fun = @(t) deal(A * sin(w * t), A * w * cos(w * t));
        R.name = sprintf('Sine %.3f m, %.3f Hz', A, f);
        R.A = A;
        R.f = f;
        R.peaks = NaN;

    case 'bump'
        H = varargin{1};
        L = varargin{2};
        v = varargin{3};
        t0 = varargin{4};
        d = L / v;
        R.fun = @(t) deal( ...
            ((t >= t0) & (t <= t0 + d)) .* (H / 2 * (1 - cos(2 * pi * (t - t0) / d))), ...
            ((t >= t0) & (t <= t0 + d)) .* (H / 2 * (2 * pi / d) * sin(2 * pi * (t - t0) / d)));
        R.name = sprintf('Bump %.0f mm x %.1f m at %.0f km/h', 1e3 * H, L, 3.6 * v);
        R.peaks = NaN;

    case 'iso8608'
        cls = upper(varargin{1});
        v = varargin{2};
        seed = 1;
        if numel(varargin) >= 3
            seed = varargin{3};
        end
        Gd0 = struct('A', 16e-6, 'B', 64e-6, 'C', 256e-6, 'D', 1024e-6, 'E', 4096e-6);
        if ~isfield(Gd0, cls)
            error('qc_road:iso', 'ISO 8608 class must be A..E.');
        end
        n0 = 0.1;                         % cycles/m reference
        N = 200;
        n = linspace(0.011, 2.83, N)';    % spatial frequency band of ISO 8608
        dn = n(2) - n(1);
        Gd = Gd0.(cls) * (n / n0) .^ (-2);
        amp = sqrt(2 * Gd * dn);
        phi = 2 * pi * mod((1:N)' * 0.6180339887498949 + seed * 0.7548776662466927, 1);
        w = 2 * pi * n * v;               % rad/s
        R.fun = @(t) deal(sum(amp .* sin(w * t + phi), 1), ...
            sum(amp .* w .* cos(w * t + phi), 1));
        R.name = sprintf('ISO 8608 class %s at %.0f km/h', cls, 3.6 * v);
        R.peaks = NaN;

    otherwise
        error('qc_road:kind', 'Unknown road "%s".', kind);
end
R.kind = lower(kind);
end

function tab = qc_road_table()
% Na et al. Table IV: peaks, amplitude (m), frequency (Hz)
tab = [3 0.025 0.188; 4 0.043 0.204; 5 0.043 0.243; 6 0.043 0.307; ...
       7 0.043 0.361; 8 0.043 0.407; 9 0.043 0.423; 10 0.043 0.505];
end
