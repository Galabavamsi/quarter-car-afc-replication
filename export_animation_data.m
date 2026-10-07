function files = export_animation_data()
%EXPORT_ANIMATION_DATA Write the trajectories the Blender scene animates.
%   files = export_animation_data() runs passive, AFC (Case A) and AFC-AW on
%   the calibrated plant for two roads and writes one CSV per road to
%   animation/:
%       anim_bump.csv    50 mm x 2.5 m bump at 20 km/h (transient)
%       anim_case10.csv  paper road Case 10 (0.043 m, 0.505 Hz)
%   Columns: t, zr, mu1, then zs_*, zu_*, u_* for passive, afc, afc_aw.
%   All lengths in metres, u in volts. animation/build_quarter_car_scene.py
%   reads these files.
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'afc2'));
out = fullfile(root, 'animation');
if ~exist(out, 'dir')
    mkdir(out);
end
P = qc_params('alleyne-cal');
roads = {qc_road('bump', 0.05, 2.5, 20 / 3.6, 1.0), qc_road('case', 10)};
T = [4, 10];
names = {'anim_bump.csv', 'anim_case10.csv'};
ctrl = {qc_ctrl('passive'), qc_ctrl('afc', 'A'), qc_ctrl('afc_aw', 'A')};
tag = {'passive', 'afc', 'afc_aw'};
files = cell(1, 2);
for k = 1:2
    runs = cell(1, 3);
    for j = 1:3
        runs{j} = qc_simulate(P, roads{k}, ctrl{j}, struct('T', T(k)));
    end
    t = runs{1}.t;
    M = [t, runs{1}.zr, runs{2}.mu1];
    hdr = {'t', 'zr', 'mu1'};
    for j = 1:3
        M = [M, runs{j}.x(1, :)', runs{j}.x(3, :)', runs{j}.u]; %#ok<AGROW>
        hdr = [hdr, {['zs_', tag{j}], ['zu_', tag{j}], ['u_', tag{j}]}]; %#ok<AGROW>
    end
    rows = num2cell(M);
    files{k} = fullfile(out, names{k});
    qc_write_csv(files{k}, hdr, rows);
    fprintf('wrote %s (%d samples)\n', files{k}, size(M, 1));
end
end
