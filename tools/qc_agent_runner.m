function qc_agent_runner(cmd)
%QC_AGENT_RUNNER Let Claude run MATLAB jobs in this project through a folder.
%
%   qc_agent_runner start    poll agent_jobs\inbox every 3 s and run new jobs
%   qc_agent_runner stop     stop polling
%   qc_agent_runner status   print whether the runner is active
%   qc_agent_runner drain    blocking loop for a headless session, e.g.
%       matlab -batch "addpath('tools'); qc_agent_runner drain"
%     runs jobs as they arrive and exits after 30 idle minutes (or when a
%     file agent_jobs\STOP appears). Use this for Simulink jobs: a model
%     built inside a timer callback can hang the session.
%
%   How it works: Claude writes a MATLAB script to agent_jobs\inbox\ (it can
%   only write inside this project folder). The runner moves it to
%   agent_jobs\running\, runs it from the project root with the Command
%   Window output captured to agent_jobs\logs\<job>.log (written live, so
%   progress can be followed), appends "STATUS: OK" or "STATUS: ERROR ..."
%   with the stack, and moves the script to agent_jobs\done\. A heartbeat
%   file shows the runner is alive.
%
%   Only .m files in agent_jobs\inbox are executed. Stop the runner whenever
%   you want with  qc_agent_runner stop  (or close MATLAB).
if nargin < 1
    cmd = 'start';
end
root = fileparts(fileparts(mfilename('fullpath')));
q = fullfile(root, 'agent_jobs');
dirs = {'inbox', 'running', 'done', 'logs'};
for i = 1:numel(dirs)
    if ~exist(fullfile(q, dirs{i}), 'dir')
        mkdir(fullfile(q, dirs{i}));
    end
end
existing = timerfindall('Tag', 'qc_agent_runner');
switch lower(cmd)
    case 'start'
        if ~isempty(existing)
            stop(existing); delete(existing);
        end
        t = timer('Tag', 'qc_agent_runner', 'ExecutionMode', 'fixedSpacing', 'Period', 3, ...
            'BusyMode', 'drop', 'TimerFcn', @(~, ~) poll_quiet(root, q));
        start(t);
        fprintf('qc_agent_runner: watching %s (stop with: qc_agent_runner stop)\n', fullfile(q, 'inbox'));
    case 'stop'
        if ~isempty(existing)
            stop(existing); delete(existing);
        end
        write_heartbeat(q, 'stopped');
        fprintf('qc_agent_runner: stopped\n');
    case 'drain'
        fprintf('qc_agent_runner: draining %s (headless loop)\n', fullfile(q, 'inbox'));
        lastWork = tic;
        while toc(lastWork) < 30 * 60 && ~exist(fullfile(q, 'STOP'), 'file')
            if poll(root, q) > 0
                lastWork = tic;
            end
            pause(3);
        end
        write_heartbeat(q, 'stopped (drain finished)');
        fprintf('qc_agent_runner: drain finished\n');
    case 'status'
        if isempty(existing)
            fprintf('qc_agent_runner: not running\n');
        else
            fprintf('qc_agent_runner: running (%s)\n', existing(1).Running);
        end
    otherwise
        error('Use start, stop, status or drain.');
end
end

function n = poll(root, q)
write_heartbeat(q, 'idle');
jobs = dir(fullfile(q, 'inbox', '*.m'));
n = numel(jobs);
if isempty(jobs)
    return;
end
[~, order] = sort({jobs.name});
jobs = jobs(order);
for k = 1:numel(jobs)
    name = jobs(k).name;
    src = fullfile(q, 'inbox', name);
    run_path = fullfile(q, 'running', name);
    movefile(src, run_path, 'f');
    [~, base] = fileparts(name);
    logf = fullfile(q, 'logs', [base, '.log']);
    write_heartbeat(q, ['running ', name]);
    old = cd(root);
    diary off;
    if exist(logf, 'file'), delete(logf); end
    diary(logf);
    fprintf('JOB %s started %s on %s\n', name, datestr(now, 31), version);
    t0 = tic;
    try
        run_job(run_path);
        fprintf('\nSTATUS: OK (%.1f s)\n', toc(t0));
    catch err
        fprintf('\nSTATUS: ERROR (%.1f s)\n%s\n', toc(t0), getReport(err, 'extended', 'hyperlinks', 'off'));
    end
    diary off;
    cd(old);
    movefile(run_path, fullfile(q, 'done', name), 'f');
    close all force;
end
write_heartbeat(q, 'idle');
end

function poll_quiet(root, q)
poll(root, q);
end

function run_job(path)
% Evaluate the job text from the project root (run() would cd into the
% job's folder). A separate function keeps job variables out of the runner.
eval(fileread(path));
end

function write_heartbeat(q, state)
fid = fopen(fullfile(q, 'heartbeat.txt'), 'w');
if fid > 0
    fprintf(fid, '%s %s\n', datestr(now, 31), state);
    fclose(fid);
end
end
