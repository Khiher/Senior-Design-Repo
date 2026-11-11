function out = yig_sim_script(Tstop)
%YIG_SIM_SCRIPT  Same plant + controller as the Simulink model, run as a
%   plain MATLAB loop.  Handy for debugging your controller with
%   breakpoints, and for running without Simulink.
%
%   out = yig_sim_script()        uses YIG / ref_profile / env_profile
%   out = yig_sim_script(Tstop)   from the base workspace (run yig_setup)
%
%   Execution order per plant step matches the Simulink model:
%     controller (every Tc, sees y from the previous plant step)
%       -> driver -> magnet -> resonator -> sensors

P   = evalin('base', 'YIG');
ref = evalin('base', 'ref_profile');
env = evalin('base', 'env_profile');
if nargin < 1, Tstop = P.sim.Tstop; end

% reset persistent state of every block
clear yig_driver_step yig_magnet_step yig_resonator_step yig_sensors_step yig_controller

Ts = P.sim.Ts;
Nc = round(P.sim.Tc / Ts);
N  = round(Tstop / Ts);
D  = P.sim.log_decim;
NL = floor(N / D);

out.t     = zeros(NL, 1);
out.ref   = zeros(NL, 1);
out.u     = zeros(NL, 6);
out.y     = zeros(NL, 8);
out.truth = zeros(NL, 11);   % [drv(5) mag(3) res(3)]

y_prev = zeros(8, 1);
u      = zeros(6, 1);
f_t    = ref(1, 2);
li = 0;
for k = 0:N-1
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
