function build_yig_model(root)
%BUILD_YIG_MODEL  Programmatically creates yig_tuning_model.slx.
%
%   Top level
%     [Target Frequency] --> [YIG Controller] --> [RT u] --> [YIG Plant] --> y
%            ^                    ^                              |
%            |                    +------ [RT y] <-- [ADC latency]
%     [Environment] (temperature, stray field) -----------------^
%
%   YIG Plant subsystem
%     u --> [Pulse Driver] --> [AlNiCo Magnet] --> [Gap Field and YIG Filter] --> [Sensors] --> y
%
%   Every grey box is a MATLAB Function block that is a one-line wrapper
%   around a function in plant/ or control/.  All of them take the
%   base-workspace struct YIG as a (non-tunable) parameter.
%
%   Re-run this any time; it deletes and rebuilds the .slx.

if nargin < 1
    root = fileparts(fileparts(mfilename('fullpath')));
end
mdl = 'yig_tuning_model';
if bdIsLoaded(mdl), close_system(mdl, 0); end
slx = fullfile(root, [mdl '.slx']);
if exist(slx, 'file'), delete(slx); end

new_system(mdl);
load_system(mdl);

%% ---- solver / config ------------------------------------------------
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
               'FixedStep', 'Ts', 'StopTime', 'Tstop');
try
    set_param(mdl, 'EnableMultiTasking', 'off');
catch
    try set_param(mdl, 'SolverMode', 'SingleTasking'); catch, end
end
try set_param(mdl, 'SingleTaskRateTransMsg', 'none'); catch, end
try set_param(mdl, 'ReturnWorkspaceOutputs', 'on'); catch, end
try set_param(mdl, 'SaveOutput', 'off', 'SaveTime', 'off'); catch, end
% make the model self-contained: load the environment if needed
set_param(mdl, 'InitFcn', ...
    'if ~exist(''YIG'',''var'') || ~exist(''ref_profile'',''var''), yig_setup; end');

%% ---- top-level blocks -----------------------------------------------
add_block('simulink/Sources/From Workspace', [mdl '/Target Frequency'], ...
    'VariableName', 'ref_profile', 'SampleTime', 'Tc', 'Interpolate', 'off', ...
    'OutputAfterFinalValue', 'Holding final value', ...
    'Position', [40 100 140 130]);

add_block('simulink/Sources/From Workspace', [mdl '/Environment'], ...
    'VariableName', 'env_profile', 'SampleTime', 'Ts', 'Interpolate', 'on', ...
    'OutputAfterFinalValue', 'Holding final value', ...
    'Position', [300 240 400 270]);

ctrl = [mdl '/YIG Controller'];
add_block('simulink/User-Defined Functions/MATLAB Function', ctrl, ...
    'Position', [220 85 360 175]);
set_mfb(ctrl, { ...
    'function u = fcn(f_target, y, YIG)'
    '%#codegen'
    '% YIG Controller block.'
    '% This is only a wrapper -- put your control law in control/yig_controller.m'
    '% (or change the line below to call a different function with the'
    '%  same signature:  u = my_ctrl(f_target, y, YIG) ).'
    '%   f_target : desired centre frequency [Hz]'
    '%   y (8x1)  : measurements, see yig_sensors_step.m'
    '%   u (6x1)  : [V0_set; fire; polarity; I_lim; I_trim; f_probe]'
    'u = yig_controller(f_target, y, YIG);'
    }, {'u', '[6 1]'});
try set_param(ctrl, 'BackgroundColor', 'yellow'); catch, end

add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT u'], ...
    'OutPortSampleTime', 'Ts', 'Position', [410 115 450 145]);

plant = [mdl '/YIG Plant'];
add_block('built-in/Subsystem', plant, 'Position', [520 90 680 280]);
build_plant(plant);
try set_param(plant, 'BackgroundColor', 'lightBlue'); catch, end

add_block('simulink/Discrete/Unit Delay', [mdl '/ADC latency'], ...
    'SampleTime', 'Ts', 'InitialCondition', '0', ...
    'Position', [560 340 600 380], 'Orientation', 'left');

add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT y'], ...
    'OutPortSampleTime', 'Tc', 'Position', [400 345 440 375], ...
    'Orientation', 'left');

% logging
tw = 'simulink/Sinks/To Workspace';
add_block(tw, [mdl '/log_ref'],   'VariableName', 'log_ref',   'SaveFormat', 'Timeseries', 'Position', [220 20 300 45]);
add_block(tw, [mdl '/log_u'],     'VariableName', 'log_u',     'SaveFormat', 'Timeseries', 'Position', [420 20 500 45]);
add_block(tw, [mdl '/log_y'],     'VariableName', 'log_y',     'SaveFormat', 'Timeseries', 'Position', [220 420 300 445]);
add_block(tw, [mdl '/log_truth'], 'VariableName', 'log_truth', 'SaveFormat', 'Timeseries', ...
    'Decimation', 'YIG.sim.log_decim', 'Position', [780 150 860 175]);
for b = {'log_ref', 'log_u', 'log_y', 'log_truth'}
    try set_param([mdl '/' b{1}], 'MaxDataPoints', 'inf'); catch, end
end

add_block('simulink/Sinks/Scope', [mdl '/Scope'], 'Position', [780 210 830 290]);
try set_param([mdl '/Scope'], 'NumInputPorts', '3'); catch, end

%% ---- top-level wiring -----------------------------------------------
L = @(a, b) add_line(mdl, a, b, 'autorouting', 'on');
L('Target Frequency/1', 'YIG Controller/1');
L('Target Frequency/1', 'log_ref/1');
L('Target Frequency/1', 'Scope/1');
L('YIG Controller/1',   'RT u/1');
L('YIG Controller/1',   'log_u/1');
L('RT u/1',             'YIG Plant/1');
L('Environment/1',      'YIG Plant/2');
L('YIG Plant/1',        'ADC latency/1');
L('ADC latency/1',      'RT y/1');
L('RT y/1',             'YIG Controller/2');
L('RT y/1',             'log_y/1');
L('YIG Plant/2',        'log_truth/1');
L('YIG Plant/3',        'Scope/2');
L('YIG Plant/4',        'Scope/3');

% captions under the key blocks
try set_param(ctrl,  'AttributesFormatString', 'edit: control\yig_controller.m'); catch, end
try set_param(plant, 'AttributesFormatString', 'params: YIG = yig_params()'); catch, end
set_param(mdl, 'Description', sprintf(['YIG tuning model.\n' ...
    'Control law: control/yig_controller.m\n' ...
    'Parameters : params/yig_params.m (base-workspace struct YIG)\n' ...
    'Run        : out = sim(''yig_tuning_model''); yig_plot_results(out)']));

save_system(mdl, slx);
open_system(mdl);
fprintf('Saved %s\n', slx);
end

%% ======================================================================
function build_plant(sys)
% remove the default In1 -> Out1 of a fresh subsystem
try
    delete_line(sys, 'In1/1', 'Out1/1');
    delete_block([sys '/In1']);
    delete_block([sys '/Out1']);
catch
end

add_block('simulink/Sources/In1', [sys '/u'],   'Position', [20  73  50  87]);
add_block('simulink/Sources/In1', [sys '/env'], 'Position', [20 253  50 267]);

drv = [sys '/Pulse Driver'];
add_block('simulink/User-Defined Functions/MATLAB Function', drv, 'Position', [110 50 250 130]);
set_mfb(drv, { ...
    'function [drv, I_coil] = fcn(u, YIG)'
    '%#codegen'
    '% Charger + cap + H-bridge + main coil + trim coil.  See plant/yig_driver_step.m'
    '% drv = [I_coil; Vc; busy; ready; I_trim]'
    'drv = yig_driver_step(u, YIG);'
    'I_coil = drv(1);'
    }, {'drv', '[5 1]'; 'I_coil', '1'});

mag = [sys '/AlNiCo Magnet'];
add_block('simulink/User-Defined Functions/MATLAB Function', mag, 'Position', [310 100 440 150]);
set_mfb(mag, { ...
    'function mag = fcn(I_coil, YIG)'
    '%#codegen'
    '% Preisach hysteresis + self-demag.  See plant/yig_magnet_step.m'
    '% mag = [m; H_eff; H_coil]'
    'mag = yig_magnet_step(I_coil, YIG);'
    }, {'mag', '[3 1]'});

res = [sys '/Gap Field and YIG Filter'];
add_block('simulink/User-Defined Functions/MATLAB Function', res, 'Position', [500 60 660 230]);
set_mfb(res, { ...
    'function [res, f0] = fcn(drv, mag, u, env, YIG)'
    '%#codegen'
    '% Gap field -> YIG resonance -> filter response.  See plant/yig_resonator_step.m'
    '% res = [f0; B_gap; P_out_dBm]'
    'res = yig_resonator_step(drv, mag, u, env, YIG);'
    'f0 = res(1);'
    }, {'res', '[3 1]'; 'f0', '1'});

sen = [sys '/Sensors'];
add_block('simulink/User-Defined Functions/MATLAB Function', sen, 'Position', [720 60 850 160]);
set_mfb(sen, { ...
    'function y = fcn(drv, res, env, YIG)'
    '%#codegen'
    '% Noisy, quantised measurements.  See plant/yig_sensors_step.m'
    'y = yig_sensors_step(drv, res, env, YIG);'
    }, {'y', '[8 1]'});

tru = [sys '/Truth'];
add_block('simulink/User-Defined Functions/MATLAB Function', tru, 'Position', [720 200 800 280]);
set_mfb(tru, { ...
    'function truth = fcn(drv, mag, res, YIG)'
    '%#codegen'
    '% Logged plant states: [drv(5); mag(3); res(3)]'
    'truth = [drv(:); mag(:); res(:)];'
    }, {'truth', '[11 1]'});

add_block('simulink/Sinks/Out1', [sys '/y'],      'Position', [920 103 950 117]);
add_block('simulink/Sinks/Out1', [sys '/truth'],  'Position', [920 233 950 247]);
add_block('simulink/Sinks/Out1', [sys '/f0'],     'Position', [920 303 950 317]);
add_block('simulink/Sinks/Out1', [sys '/I_coil'], 'Position', [920 353 950 367]);

L = @(a, b) add_line(sys, a, b, 'autorouting', 'on');
L('u/1',            'Pulse Driver/1');
L('u/1',            'Gap Field and YIG Filter/3');
L('env/1',          'Gap Field and YIG Filter/4');
L('env/1',          'Sensors/3');
L('Pulse Driver/1', 'Gap Field and YIG Filter/1');
L('Pulse Driver/1', 'Sensors/1');
L('Pulse Driver/1', 'Truth/1');
L('Pulse Driver/2', 'AlNiCo Magnet/1');
L('Pulse Driver/2', 'I_coil/1');
L('AlNiCo Magnet/1','Gap Field and YIG Filter/2');
L('AlNiCo Magnet/1','Truth/2');
L('Gap Field and YIG Filter/1', 'Sensors/2');
L('Gap Field and YIG Filter/1', 'Truth/3');
L('Gap Field and YIG Filter/2', 'f0/1');
L('Sensors/1',      'y/1');
L('Truth/1',        'truth/1');
end

%% ======================================================================
function set_mfb(blk, lines, outs)
% Put code into a MATLAB Function block, turn its "YIG" argument into
% a non-tunable parameter (looked up from the base workspace), and fix the
% size of every output.  outs = {'name', '[n 1]'; ...}.  Explicit output
% sizes are needed because the controller/plant feedback loop leaves
% Simulink's size propagation underspecified otherwise.
code = sprintf('%s\n', lines{:});
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', blk);
if isempty(ch)
    error('build_yig_model:noChart', 'Could not find MATLAB Function block %s', blk);
end
ch.Script = code;
d = ch.find('-isa', 'Stateflow.Data', 'Name', 'YIG');
d.Scope = 'Parameter';
try
    d.Tunable = false;
catch
    try d.Props.Tunable = false; catch, end
end
for k = 1:size(outs, 1)
    o = ch.find('-isa', 'Stateflow.Data', 'Name', outs{k,1});
    o.Props.Array.Size = outs{k,2};
end
end
