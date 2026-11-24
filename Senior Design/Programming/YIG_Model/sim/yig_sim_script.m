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

out = yig_sim_chunk(P, ref, env, round(Tstop / P.sim.Ts));
end
