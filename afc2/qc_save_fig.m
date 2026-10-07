function qc_save_fig(fig, basepath)
%QC_SAVE_FIG Save a figure as vector PDF + 200 dpi PNG.
%   MATLAB: forces the light graphics theme (R2025a+ figures otherwise
%   follow a dark desktop theme) and uses exportgraphics when available.
%   Octave: falls back to print.
[folder, ~, ~] = fileparts(basepath);
if ~isempty(folder) && ~exist(folder, 'dir')
    mkdir(folder);
end
isMatlab = exist('OCTAVE_VERSION', 'builtin') == 0;
if isMatlab && isprop(fig, 'Theme')
    try
        theme(fig, 'light');
    catch
        try
            fig.Theme = 'light';
        catch
        end
    end
    drawnow;
end
if isMatlab && exist('exportgraphics') > 0 %#ok<EXIST> builtin, p-file or m-file
    exportgraphics(fig, [basepath, '.pdf'], 'ContentType', 'vector');
    exportgraphics(fig, [basepath, '.png'], 'Resolution', 200);
else
    print(fig, [basepath, '.png'], '-dpng', '-r150');
    print(fig, [basepath, '.pdf'], '-dpdf');
end
end
