function ref = qc_paper_ref()
%QC_PAPER_REF Numbers reported by Na et al. (2022), for side-by-side tables.
%   Case-10 indices (Fig. 7 / Table II), accelerations (Fig. 8) and
%   computation times (Table III) are PRINTED values.
%   The road-sweep trends (Fig. 12) and control-voltage trends (Fig. 13)
%   are READ OFF THE PLOTS BY EYE from a 250 dpi render (approx. +/-3% of
%   full scale) and are flagged as approximate everywhere they are used.
%   The fixed-waveform test (Fig. 5) is a ~0.043 m, ~0.5 Hz, 10-peak
%   sinusoid-like pneumatic excitation, i.e. the same condition as road
%   Case 10 (Fig. 11b / Table IV), so Case 10 is compared against it.
ref.controllers = {'AFC', 'BSC', 'PID'};
% ---- printed (Fig. 7, Table II rounds these) -------------------------------
ref.IAE = [0.1216, 0.2108, 0.1881];
ref.ITAE = [1.1396, 1.9522, 1.1907];
ref.ITSE = [0.0093, 0.0224, 0.0219];
ref.acc_rms = [1.6651, 1.7246, 1.7533];   % Fig. 8
ref.acc_max = [7.21, 7.225, 7.725];       % Fig. 8
ref.comp_time_ms = [4.2, 5.8, 2.6];       % Table III, Kinetis MK60D
% ---- approximate plot readings (Fig. 12 / Fig. 13) ------------------------
ref.sweep.peaks = 3:10;
%                3      4      5      6      7      8      9      10
ref.sweep.IAE = [0.083  0.085  0.088  0.102  0.100  0.127  0.150  0.1216;   % AFC
                 0.160  0.193  0.208  0.220  0.237  0.242  0.262  0.2108;   % BSC
                 0.148  0.207  0.200  0.242  0.253  0.263  0.262  0.1881];  % PID
ref.sweep.ITAE = [0.85  0.72   0.82   0.98   0.92   1.10   1.25   1.1396;
                  1.95  2.05   2.15   2.28   2.50   2.55   2.70   1.9522;
                  1.65  2.13   2.03   2.30   2.62   2.68   3.05   1.1907];
ref.sweep.ITSE = [0.0055 0.0035 0.0045 0.0060 0.0060 0.0080 0.0105 0.0093;
                  0.0190 0.0230 0.0250 0.0280 0.0335 0.0345 0.0415 0.0224;
                  0.0150 0.0290 0.0240 0.0320 0.0400 0.0430 0.0540 0.0219];
ref.sweep.u_rms = [0.45  0.42   0.42   0.47   0.50   0.58   0.68   0.50;
                   2.15  2.85   2.93   3.10   3.28   3.18   3.65   4.72;
                   2.20  2.72   2.65   3.03   3.00   3.00   3.25   5.00];
ref.sweep.u_max = [1.90  2.65   1.52   1.68   2.03   2.25   2.28   2.30;
                   5.00  5.00   5.00   5.00   5.00   5.00   5.00   5.00;
                   4.72  5.00   5.00   5.00   5.00   4.72   5.00   5.00];
ref.u_rms_approx = ref.sweep.u_rms(:, end)';
ref.u_max_approx = ref.sweep.u_max(:, end)';
ref.notes = ['Case-10 IAE/ITAE/ITSE and accelerations are printed values ' ...
    '(Fig. 7/8). Sweep curves and voltages are by-eye readings of Figs. 12-13.'];
end
