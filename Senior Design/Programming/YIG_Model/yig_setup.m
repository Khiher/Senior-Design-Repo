%YIG_SETUP  Run this first.  Sets up the MATLAB environment for the YIG
%   tuning model:
%     1. adds the project folders to the path
%     2. loads all parameters into the base-workspace struct  YIG
%     3. defines the test profiles (frequency targets, temperature,
%        stray field) used by both the Simulink model and the script sim
%     4. builds yig_tuning_model.slx if it does not exist yet
%
%   After it finishes:
%       out = sim('yig_tuning_model');  yig_plot_results(out)
%   or, without Simulink:
%       out = yig_sim_script();         yig_plot_results(out)

root = fileparts(mfilename('fullpath'));
addpath(root, fullfile(root,'params'), fullfile(root,'plant'), ...
        fullfile(root,'control'), fullfile(root,'sim'));

%% ---- parameters -------------------------------------------------------
YIG   = yig_params();           % <- the struct every block receives
Ts    = YIG.sim.Ts;             % plant step
Tc    = YIG.sim.Tc;             % controller step
Tstop = YIG.sim.Tstop;

%% ---- test profiles ----------------------------------------------------
% Target centre frequency: [time(s)  f_target(Hz)]  (piecewise constant)
ref_profile = [ 0.00  10.00e9
                0.08  14.00e9
                0.16   5.00e9
                0.24   5.02e9      % small move -> handled by the trim coil only
                0.30  12.00e9 ];

% Environment: [time(s)  temperature(degC)  stray field(T)]  (interpolated)
env_profile = [ 0.000  25  0
                0.050  25  0
                0.200  31  0
                0.200  31  0.5e-3  % 0.5 mT stray-field step (~14 MHz)
                0.400  37  0.5e-3 ];

%% ---- Simulink model ---------------------------------------------------
mdl = 'yig_tuning_model';
if exist('new_system', 'builtin') || exist('new_system', 'file')
    if ~exist(fullfile(root, [mdl '.slx']), 'file')
        fprintf('Building %s.slx ...\n', mdl);
        build_yig_model(root);
    end
    fprintf(['YIG environment ready.\n' ...
             '  Simulink : out = sim(''%s''); yig_plot_results(out)\n' ...
             '  Script   : out = yig_sim_script(); yig_plot_results(out)\n' ...
             '  Your control law lives in control/yig_controller.m\n'], mdl);
else
    fprintf(['YIG environment ready (Simulink not found).\n' ...
             '  Script sim : out = yig_sim_script(); yig_plot_results(out)\n']);
end
