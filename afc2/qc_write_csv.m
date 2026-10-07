function qc_write_csv(path, header, rows)
%QC_WRITE_CSV Write a cell array table to CSV (MATLAB/Octave portable).
%   header: 1xM cell of strings; rows: NxM cell (numbers or strings).
fid = fopen(path, 'w');
if fid < 0
    error('qc_write_csv:open', 'Cannot open %s', path);
end
cleaner = onCleanup(@() fclose(fid));
fprintf(fid, '%s\n', strjoin(header, ','));
for i = 1:size(rows, 1)
    cells = cell(1, size(rows, 2));
    for j = 1:size(rows, 2)
        v = rows{i, j};
        if ischar(v)
            cells{j} = v;
        elseif isempty(v) || (isscalar(v) && isnan(v))
            cells{j} = '';
        else
            cells{j} = sprintf('%.6g', v);
        end
    end
    fprintf(fid, '%s\n', strjoin(cells, ','));
end
end
