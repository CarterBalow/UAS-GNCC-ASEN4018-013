function build_multirotor_simulink_model(modelName, forceRebuild)
%BUILD_MULTIROTOR_SIMULINK_MODEL Create the time-domain G&N Simulink model.
% The model preserves the starter's Guidance -> Point Mass -> Navigation
% architecture while adding mission phases, logging, and bounded commands.

if nargin < 1
    modelName = 'multirotor_gn_model';
end
if nargin < 2
    forceRebuild = false;
end
modelFile = fullfile(fileparts(mfilename('fullpath')),[modelName '.slx']);
if isfile(modelFile) && ~forceRebuild
    fprintf('Model already exists: %s\n',modelFile);
    return;
end
if bdIsLoaded(modelName)
    close_system(modelName,0);
end
if isfile(modelFile)
    delete(modelFile);
end

new_system(modelName);
cleanup = onCleanup(@() close_if_loaded(modelName));
open_system(modelName);
set_param(modelName,'SolverType','Fixed-step','Solver','ode3', ...
    'FixedStep','simulationSampleTime','StopTime','600', ...
    'ReturnWorkspaceOutputs','on','SignalLogging','off');

add_block('simulink/Ports & Subsystems/Subsystem',[modelName '/Guidance'], ...
    'Position',[100 105 245 235]);
add_block('simulink/Ports & Subsystems/Subsystem',[modelName '/Point Mass'], ...
    'Position',[330 110 475 205]);
add_block('simulink/Ports & Subsystems/Subsystem',[modelName '/Navigation'], ...
    'Position',[555 110 700 205]);
build_guidance([modelName '/Guidance']);
build_point_mass([modelName '/Point Mass']);
build_navigation([modelName '/Navigation']);

add_line(modelName,'Guidance/1','Point Mass/1','autorouting','on');
add_line(modelName,'Point Mass/1','Navigation/1','autorouting','on');
add_line(modelName,'Navigation/1','Guidance/1','autorouting','on');
add_block('simulink/Discrete/Zero-Order Hold',[modelName '/Sample True Position'], ...
    'SampleTime','simulationSampleTime','Position',[500 245 610 275]);
add_line(modelName,'Point Mass/1','Sample True Position/1','autorouting','on');
add_line(modelName,'Sample True Position/1','Guidance/2','autorouting','on');

% Signal logs returned as timeseries in Simulink.SimulationOutput.
add_log(modelName,'Point Mass/1','true_position_enu',[780 35 900 65]);
add_log(modelName,'Point Mass/2','true_velocity_enu',[780 75 900 105]);
add_log(modelName,'Navigation/1','estimated_position_enu',[780 115 900 145]);
add_log(modelName,'Guidance/1','velocity_command_enu',[780 155 900 185]);
add_log(modelName,'Guidance/2','target_waypoint_enu',[780 195 900 225]);
add_log(modelName,'Guidance/3','waypoint_index',[780 235 900 265]);
add_log(modelName,'Guidance/4','mission_phase',[780 275 900 305]);
add_log(modelName,'Guidance/5','mission_complete',[780 315 900 345]);

add_block('simulink/Sinks/Stop Simulation',[modelName '/Stop on Mission Complete'], ...
    'Position',[760 370 900 400]);
add_line(modelName,'Guidance/5','Stop on Mission Complete/1','autorouting','on');

note = sprintf(['Coordinate frame: local ENU [East, North, Up] in meters\n' ...
    'Run from MATLAB with: results = run_multirotor_mission();']);
add_block('simulink/Model-Wide Utilities/DocBlock',[modelName '/Read Me'], ...
    'Position',[100 300 330 355]);
set_param([modelName '/Read Me'],'DocumentType','Text','UserData',note);

Simulink.BlockDiagram.arrangeSystem(modelName);
save_system(modelName,modelFile);
fprintf('Created %s\n',modelFile);
clear cleanup;
end

function build_guidance(path)
clear_subsystem(path);
add_block('simulink/Sources/In1',[path '/Estimated Position ENU'], ...
    'PortDimensions','3','Position',[25 35 55 55]);
add_block('simulink/Sources/In1',[path '/True Position ENU'], ...
    'Port','2','PortDimensions','3','Position',[25 75 55 95]);
add_block('simulink/Sources/Constant',[path '/Route ENU'], ...
    'Value','routeWaypointsPadded','Position',[25 120 100 150]);
add_block('simulink/Sources/Constant',[path '/Route Phase'], ...
    'Value','routePhasePadded','Position',[25 165 100 195]);
add_block('simulink/Sources/Constant',[path '/Route Count'], ...
    'Value','routeCount','Position',[25 210 100 240]);
add_block('simulink/Sources/Constant',[path '/Guidance Parameters'], ...
    'Value','guidanceParams','Position',[25 255 100 285]);
block = [path '/Waypoint Guidance'];
add_block('simulink/User-Defined Functions/MATLAB Function',block, ...
    'Position',[175 55 410 280]);
chart = find(sfroot,'-isa','Stateflow.EMChart','Path',block);
chart.Script = sprintf([ ...
    'function [vCmd,target,wpIndex,phase,complete] = fcn(pos,truePos,route,phases,count,params)\n' ...
    '%%#codegen\n' ...
    '[vCmd,target,wpIndex,phase,complete] = guidance_step(pos,truePos,route,phases,count,params);\n' ...
    'end\n']);
outNames = {'Velocity Command ENU','Target Waypoint ENU','Waypoint Index','Mission Phase','Mission Complete'};
for k = 1:numel(outNames)
    add_block('simulink/Sinks/Out1',[path '/' outNames{k}], ...
        'Port',num2str(k),'Position',[455 35+43*(k-1) 485 55+43*(k-1)]);
end
add_line(path,'Estimated Position ENU/1','Waypoint Guidance/1');
add_line(path,'True Position ENU/1','Waypoint Guidance/2');
add_line(path,'Route ENU/1','Waypoint Guidance/3');
add_line(path,'Route Phase/1','Waypoint Guidance/4');
add_line(path,'Route Count/1','Waypoint Guidance/5');
add_line(path,'Guidance Parameters/1','Waypoint Guidance/6');
for k = 1:5
    add_line(path,sprintf('Waypoint Guidance/%d',k),sprintf('%s/1',outNames{k}));
end
end

function build_point_mass(path)
clear_subsystem(path);
add_block('simulink/Sources/In1',[path '/Velocity Command ENU'], ...
    'PortDimensions','3','Position',[25 90 55 110]);
add_block('simulink/Signal Routing/Demux',[path '/Demux ENU'], ...
    'Outputs','3','Position',[95 45 105 155]);
axisNames = {'East','North','Up'};
for k = 1:3
    add_block('simulink/Continuous/Transfer Fcn',[path '/' axisNames{k} ' Velocity Response'], ...
        'Numerator','1','Denominator','[vehicleTimeConstantSec 1]', ...
        'Position',[150 35+50*(k-1) 250 65+50*(k-1)]);
    add_line(path,sprintf('Demux ENU/%d',k),sprintf('%s Velocity Response/1',axisNames{k}));
end
add_block('simulink/Signal Routing/Mux',[path '/Velocity Mux ENU'], ...
    'Inputs','3','Position',[315 35 325 145]);
add_block('simulink/Signal Routing/Mux',[path '/Position Mux ENU'], ...
    'Inputs','3','Position',[430 175 440 285]);
for k = 1:3
    integrator = [axisNames{k} ' Position'];
    add_block('simulink/Continuous/Integrator',[path '/' integrator], ...
        'InitialCondition','0','Position',[315 175+50*(k-1) 365 205+50*(k-1)]);
    if k == 3
        set_param([path '/' integrator],'LimitOutput','on', ...
            'LowerSaturationLimit','0','UpperSaturationLimit','inf');
    end
    add_line(path,sprintf('%s Velocity Response/1',axisNames{k}), ...
        sprintf('Velocity Mux ENU/%d',k));
    add_line(path,sprintf('%s Velocity Response/1',axisNames{k}), ...
        sprintf('%s/1',integrator));
    add_line(path,sprintf('%s/1',integrator),sprintf('Position Mux ENU/%d',k));
end
add_block('simulink/Sinks/Out1',[path '/True Position ENU'], ...
    'Position',[500 210 530 230]);
add_block('simulink/Sinks/Out1',[path '/True Velocity ENU'], ...
    'Port','2','Position',[500 80 530 100]);
add_line(path,'Velocity Command ENU/1','Demux ENU/1');
add_line(path,'Position Mux ENU/1','True Position ENU/1');
add_line(path,'Velocity Mux ENU/1','True Velocity ENU/1');
end

function build_navigation(path)
clear_subsystem(path);
add_block('simulink/Sources/In1',[path '/True Position ENU'], ...
    'PortDimensions','3','Position',[25 70 55 90]);
add_block('simulink/Sources/Random Number',[path '/Position Sensor Noise'], ...
    'Mean','[0;0;0]','Variance','navigationNoiseVariance*[1;1;1]', ...
    'Seed','navigationNoiseSeeds','SampleTime','simulationSampleTime', ...
    'Position',[25 130 100 165]);
add_block('simulink/Math Operations/Sum',[path '/Add Sensor Noise'], ...
    'Inputs','++','Position',[155 70 180 105]);
add_block('simulink/Discrete/Discrete Filter',[path '/Navigation Low Pass Filter'], ...
    'Numerator','navigationFilterCoefficient', ...
    'Denominator','[1 -(1-navigationFilterCoefficient)]', ...
    'SampleTime','simulationSampleTime','Position',[230 65 350 110]);
add_block('simulink/Sinks/Out1',[path '/Estimated Position ENU'], ...
    'Position',[410 75 440 95]);
add_line(path,'True Position ENU/1','Add Sensor Noise/1');
add_line(path,'Position Sensor Noise/1','Add Sensor Noise/2');
add_line(path,'Add Sensor Noise/1','Navigation Low Pass Filter/1');
add_line(path,'Navigation Low Pass Filter/1','Estimated Position ENU/1');
end

function add_log(model,source,varName,position)
blockName = strrep(varName,'_',' ');
add_block('simulink/Sinks/To Workspace',[model '/' blockName], ...
    'VariableName',varName,'SaveFormat','Timeseries','Position',position);
add_line(model,source,[blockName '/1'],'autorouting','on');
end

function clear_subsystem(path)
Simulink.SubSystem.deleteContents(path);
end

function close_if_loaded(modelName)
if bdIsLoaded(modelName)
    close_system(modelName,0);
end
end
