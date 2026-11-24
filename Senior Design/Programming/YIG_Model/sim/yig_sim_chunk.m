function [out, S] = yig_sim_chunk(P, ref, env, N, S)
%YIG_SIM_CHUNK  Run the next N plant steps and keep the state.
%
%   [out, S] = yig_sim_chunk(P, ref, env, N)      start a new run
%   [out, S] = yig_sim_chunk(P, ref, env, N, S)   carry on from S
%
%   The plant and controller keep their state in persistent variables, so
%   only one run can be in progress at a time.  ref and env are the
%   ref_profile / env_profile tables, in absolute time since the run began.
%   yig_sim_script() is one chunk; the web demo runs one chunk per command.

if nargin < 5
    % reset persistent state of every block
    clear yig_driver_step yig_magnet_step yig_resonator_step yig_sensors_step yig_controller
    S.k = 0;
    S.y_prev = zeros(8, 1);
    S.u = zeros(8, 1);
    S.f_t = ref(1, 2);
end

Ts = P.sim.Ts;
Nc = round(P.sim.Tc / Ts);
D  = P.sim.log_decim;
NL = floor(N / D);

out.t     = zeros(NL, 1);
out.ref   = zeros(NL, 1);
out.u     = zeros(NL, 8);
out.y     = zeros(NL, 8);
out.truth = zeros(NL, 11);   % [drv(5) mag(3) res(3)]

y_prev = S.y_prev;  u = S.u;  f_t = S.f_t;
li = 0;
for k = S.k:S.k + N - 1
    t = k*Ts;
    if mod(k, Nc) == 0
        f_t = ref(find(ref(:,1) <= t + 1e-12, 1, 'last'), 2);
        u = yig_controller(f_t, y_prev, P);
    end
    e   = env_at(env, t);
    drv = yig_driver_step(u, P);
    mag = yig_magnet_step(drv(1), P);
    res = yig_resonator_step(drv, mag, u, e, P);
    y   = yig_sensors_step(drv, res, e, P);
    if mod(k, D) == 0 && li < NL
        li = li + 1;
        out.t(li) = t;  out.ref(li) = f_t;
        out.u(li,:) = u';  out.y(li,:) = y';
        out.truth(li,:) = [drv; mag; res]';
    end
    y_prev = y;
end
out.source = 1;   % 1 = script sim
S.k = S.k + N;  S.y_prev = y_prev;  S.u = u;  S.f_t = f_t;
end

function e = env_at(env, t)
% linear interpolation; for duplicate time stamps take the later row
k = find(env(:,1) <= t, 1, 'last');
if isempty(k), e = env(1, 2:3)'; return; end
if k == size(env, 1), e = env(k, 2:3)'; return; end
t0 = env(k,1);  t1 = env(k+1,1);
a = (t - t0) / (t1 - t0);
e = ((1-a)*env(k, 2:3) + a*env(k+1, 2:3))';
end
