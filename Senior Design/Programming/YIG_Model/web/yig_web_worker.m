function yig_web_worker(workdir)
%YIG_WEB_WORKER  MATLAB side of the web demo (started by server.py).
%
%   Polls <workdir>/job.json, runs the job on the real model and writes
%   <workdir>/result.json.  Jobs:
%     session   board, strategy, seed          fit the magnet, start a fresh live run
%     step      f_GHz, T, B_mT, dur_ms         next chunk of the live run (script sim)
%     simulink  board, strategy                the full test profile in Simulink
%   Quits when the server's heartbeat file goes stale.

web  = fileparts(mfilename('fullpath'));
root = fileparts(web);
addpath(root, web, fullfile(root,'params'), fullfile(root,'plant'), ...
        fullfile(root,'control'), fullfile(root,'sim'), fullfile(root,'devices'));
cache = fullfile(web, '.cache');
if ~exist(cache, 'dir'), mkdir(cache); end
has_slk = exist('sim', 'builtin') || exist('sim', 'file');
if has_slk
    Simulink.fileGenControl('set', 'CacheFolder', cache, 'CodeGenFolder', cache, 'createDir', true);
end
job_f = fullfile(workdir, 'job.json');
hb_f  = fullfile(workdir, 'heartbeat');
write_json(fullfile(workdir, 'ready.json'), struct('version', version('-release'), 'simulink', logical(has_slk)));

L = [];                                   % live session
n_stale = 0;
while true
    if isfile(job_f)
        job = jsondecode(fileread(job_f));
        delete(job_f);
        try
            switch job.cmd
                case 'session',  [r, L] = start_session(job, root, cache);
                case 'step',     [r, L] = run_step(job, L);
                % the Simulink blocks keep their own state, so a full test
                % leaves the live session untouched
                case 'simulink', r = run_simulink(job, root, cache);
                otherwise, error('unknown job "%s"', job.cmd);
            end
            r.ok = true;
        catch err
            r = struct('ok', false, 'error', err.message);
            fprintf('%s\n', getReport(err, 'extended', 'hyperlinks', 'off'));
        end
        r.id = job.id;
        write_json(fullfile(workdir, 'result.json'), r);
    end
    % after the laptop sleeps the heartbeat looks old until the server wakes
    % too, so only quit if it stays old for a while
    h = dir(hb_f);
    if isempty(h) || (now - h.datenum)*86400 > 20
        n_stale = n_stale + 1;
        if n_stale > 400, break; end      % ~15 s of polling
    else
        n_stale = 0;
    end
    pause(0.03);
end
end

%% ---- jobs -------------------------------------------------------------
function [r, L] = start_session(job, root, cache)
[P, dev] = load_board(job.board, root, cache);
P.ctrl.strategy = job.strategy;
rng(job.seed);
L.P = P;  L.S = [];  L.t = 0;  L.env = [0 25 0];  L.ref = [];
r.board = board_info(job.board, dev, P);
end

function [r, L] = run_step(job, L)
if isempty(L), error('no session'); end
P = L.P;
N = round(job.dur_ms*1e-3/P.sim.Ts);
t0 = L.t;
L.ref = [L.ref; t0, job.f_GHz*1e9];
if job.T ~= L.env(end,2) || job.B_mT*1e-3 ~= L.env(end,3)
    % step change in the environment (the previous value holds until t0)
    L.env = [L.env; t0, L.env(end,2:3); t0, job.T, job.B_mT*1e-3];
end
tic;
if isempty(L.S)
    [o, L.S] = yig_sim_chunk(P, L.ref, L.env, N);
else
    [o, L.S] = yig_sim_chunk(P, L.ref, L.env, N, L.S);
end
r.wall_s = toc;
L.t = L.S.k*P.sim.Ts;
r.run  = pack_run(o, 4000);
r.spec = pack_spec(o, P, [t0, job.f_GHz*1e9]);
r.env  = struct('T', job.T, 'B_mT', job.B_mT);
end

function r = run_simulink(job, root, cache)
[P, dev] = load_board(job.board, root, cache);
P.ctrl.strategy = job.strategy;
[ref, env] = test_profile(root);
assignin('base', 'YIG', P);
assignin('base', 'Ts', P.sim.Ts);  assignin('base', 'Tc', P.sim.Tc);
assignin('base', 'Tstop', P.sim.Tstop);
assignin('base', 'ref_profile', ref);  assignin('base', 'env_profile', env);
tic;
load_system(fullfile(root, 'yig_tuning_model.slx'));
so = sim('yig_tuning_model');
r.wall_s = toc;
vis = get(0, 'DefaultFigureVisible');  set(0, 'DefaultFigureVisible', 'off');
o = yig_plot_results(so);
close(gcf);  set(0, 'DefaultFigureVisible', vis);
r.run   = pack_run(o, 6000);
r.spec  = pack_spec(o, P, ref);
r.board = board_info(job.board, dev, P);
r.strategy = job.strategy;
r.env_profile = env;
end

%% ---- board + fit cache ------------------------------------------------
function [P, dev] = load_board(name, root, cache)
% The magnet fit takes ~25 s, so keep it until any of its inputs change.
dev = feval(name);
src = '';
d = [dir(fullfile(root,'params','*.m')); dir(fullfile(root,'plant','*.m')); ...
     dir(fullfile(root,'devices',[name '.m'])); ...
     dir(fullfile(root,'sim','yig_fit_device.m')); dir(fullfile(root,'sim','yig_calibrate_hall.m'))];
for k = 1:numel(d), src = [src fileread(fullfile(d(k).folder, d(k).name))]; end %#ok<AGROW>
f = fullfile(cache, [name '.mat']);
if isfile(f)
    c = load(f);
    if isequal(c.src, src), P = c.P; return; end
end
[P, FIT] = yig_fit_device(yig_params(dev));
P = yig_calibrate_hall(P, FIT);
save(f, 'P', 'src');
end

function [ref_profile, env_profile] = test_profile(root)
% the two profile tables, straight from yig_setup.m
txt = fileread(fullfile(root, 'yig_setup.m'));
for v = {'ref_profile', 'env_profile'}
    m = regexp(txt, [v{1} '\s*=\s*\[.*?\];'], 'match', 'once');
    eval(m);
end
end

function b = board_info(name, dev, P)
b = struct('id', name, 'name', dev.name, 'f_min', min(dev.f)/1e9, 'f_max', max(dev.f)/1e9, ...
           'V_reset', dev.V_reset, 'tol_MHz', P.ctrl.spec_tol/1e6);
end

%% ---- packing ----------------------------------------------------------
function p = pack_run(o, nmax)
% thin the log, but always keep each bucket's smallest and largest error
% so short bursts are never dropped
n = numel(o.t);
err = o.truth(:,9) - o.ref;
if n > nmax
    w = ceil(n/(nmax/2));
    keep = false(n, 1);
    for a = 1:w:n
        q = a:min(a+w-1, n);
        [~, i1] = min(err(q));  [~, i2] = max(err(q));
        keep(q([i1 i2])) = true;
    end
    md = o.u(:,8);
    keep([true; diff(md) ~= 0]) = true;   % every mode change too
    keep([1 n]) = true;
    i = find(keep);
else
    i = (1:n)';
end
p.t    = round(o.t(i)*1e3, 5);            % ms
p.ref  = round(o.ref(i)/1e9, 5);          % GHz
p.f    = round(o.truth(i,9)/1e9, 6);      % GHz (true)
p.trim = round(o.truth(i,5)*1e3, 3);      % mA
p.mode = o.u(i,8);
% pulses: rising edge of the fire command
fire = o.u(:,2) > 0.5;
e = find(fire & [true; ~fire(1:end-1)]);
kind = zeros(numel(e), 1);
for k = 1:numel(e)
    prev = find(o.u(1:e(k),8) ~= 2, 1, 'last');     % mode before PULSE_WAIT
    if ~isempty(prev), kind(k) = o.u(prev,8); end    % 1 reset, 3 write, 8 reverse
end
p.pulse = struct('t', round(o.t(e)*1e3, 5), 'V', round(o.u(e,1).*sign(o.u(e,3)), 4), 'kind', kind);
end

function q = pack_spec(o, P, ref)
R = yig_spec_report(o, P, ref, false);
t_end = [ref(2:end,1); o.t(end)];
conf = (ref(:,1) + R.t_resp) <= t_end - 1e-3;   % full 1 ms hold was seen
q = struct('f', R.f_target/1e9, 't_cmd', ref(:,1)*1e3, 't_resp', R.t_resp*1e3, ...
           'confirmed', conf, 'max_err', R.max_err/1e6, 't_out', R.t_out*1e3, ...
           'standby_frac', R.standby_frac, 'P_standby', R.P_standby*1e3);
end

function write_json(f, s)
% write then rename, so the reader never sees half a file
tmp = [f '.tmp'];
fid = fopen(tmp, 'w');  fwrite(fid, jsonencode(s));  fclose(fid);
movefile(tmp, f, 'f');
end
