function S = qc_style()
%QC_STYLE Shared colours/line styles so every figure reads the same way.
%   Colours match the v1 project figures (AFC teal, BSC orange, PID
%   purple, passive grey, bounds dark red).
S.afc = [0.000 0.396 0.502];
S.bsc = [0.773 0.329 0.106];
S.pid = [0.369 0.369 0.588];
S.passive = [0.45 0.45 0.45];
S.bound = [0.698 0.133 0.133];
S.paper = [0.85 0.85 0.85];
S.lw = 1.2;
S.font = 10;
end
