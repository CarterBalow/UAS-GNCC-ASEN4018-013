function results = run_multirotor_mission(overrides)
%RUN_MULTIROTOR_MISSION Configure, simulate, verify, and visualize one mission.
%   results = run_multirotor_mission()
%   results = run_multirotor_mission(42)              % reproducible seed
%   results = run_multirotor_mission(overridesStruct) % change settings
%
% Coordinate order throughout is [East, North, Up] in meters (ENU).

if nargin < 1
    overrides = struct();
elseif isnumeric(overrides) && isscalar(overrides)
    overrides = struct('randomSeed',overrides);
elseif ~isstruct(overrides)
    error('Input must be a scalar random seed or an overrides structure.');
end

projectFolder = fileparts(mfilename('fullpath'));
addpath(projectFolder);
cfg = mission_config(overrides);
aoi = generate_random_aoi(cfg);
mission = generate_mission_waypoints(cfg,aoi);

routeWaypointsPadded = zeros(cfg.simulation.maxWaypoints,3);
routePhasePadded = zeros(cfg.simulation.maxWaypoints,1);
routeWaypointsPadded(1:mission.routeCount,:) = mission.waypointsENU;
routePhasePadded(1:mission.routeCount) = mission.phaseByWaypoint;
routeCount = mission.routeCount;
guidanceParams = [cfg.guidance.acceptanceRadiusM; ...
    cfg.flight.transitSpeedMps; cfg.flight.searchSpeedMps; ...
    cfg.flight.climbSpeedMps; cfg.flight.descentSpeedMps; ...
    cfg.guidance.slowdownRadiusM; ...
    cfg.guidance.landingHorizontalToleranceM; ...
    cfg.guidance.landingAltitudeToleranceM];
vehicleTimeConstantSec = cfg.vehicle.velocityTimeConstantSec;
simulationSampleTime = cfg.simulation.sampleTimeSec;
navigationNoiseVariance = cfg.navigation.positionNoiseStdM^2;
navigationFilterCoefficient = cfg.navigation.filterCoefficient;
if isempty(cfg.randomSeed)
    navigationNoiseSeeds = randi([1,2^16-1],3,1);
else
    navigationNoiseSeeds = mod(round(cfg.randomSeed)+(1:3)'*997,2^31-1);
end

modelFile = fullfile(projectFolder,[cfg.simulation.modelName '.slx']);
if ~isfile(modelFile)
    build_multirotor_simulink_model(cfg.simulation.modelName);
end

simIn = Simulink.SimulationInput(cfg.simulation.modelName);
simIn = simIn.setVariable('routeWaypointsPadded',routeWaypointsPadded);
simIn = simIn.setVariable('routePhasePadded',routePhasePadded);
simIn = simIn.setVariable('routeCount',routeCount);
simIn = simIn.setVariable('guidanceParams',guidanceParams);
simIn = simIn.setVariable('vehicleTimeConstantSec',vehicleTimeConstantSec);
simIn = simIn.setVariable('simulationSampleTime',simulationSampleTime);
simIn = simIn.setVariable('navigationNoiseVariance',navigationNoiseVariance);
simIn = simIn.setVariable('navigationNoiseSeeds',navigationNoiseSeeds);
simIn = simIn.setVariable('navigationFilterCoefficient',navigationFilterCoefficient);
simIn = simIn.setModelParameter('StopTime',num2str(cfg.simulation.maxTimeSec));

fprintf('Running %s with ENU coordinates [East, North, Up] in meters.\n', ...
    cfg.simulation.modelName);
fprintf('AOI center: [%.2f, %.2f] m; size: %.2f x %.2f m; route: %d waypoints.\n', ...
    aoi.centerEN(1),aoi.centerEN(2),aoi.widthM,aoi.heightM,mission.routeCount);
simOut = sim(simIn);

results.timeSec = signal_time(simOut,'true_position_enu');
results.positionENU = signal_data(simOut,'true_position_enu');
results.velocityENU = signal_data(simOut,'true_velocity_enu');
results.estimatedPositionENU = signal_data(simOut,'estimated_position_enu');
results.velocityCommandENU = signal_data(simOut,'velocity_command_enu');
results.targetENU = signal_data(simOut,'target_waypoint_enu');
results.waypointIndex = signal_data(simOut,'waypoint_index');
results.phaseCode = signal_data(simOut,'mission_phase');
results.missionComplete = signal_data(simOut,'mission_complete');
results.config = cfg;
results.aoi = aoi;
results.mission = mission;
results.plotLimits = compute_mission_plot_limits( ...
    results.positionENU,mission,aoi,cfg);
results.checks = check_mission_results(results,cfg,mission,aoi);

print_check_summary(results.checks,cfg);
if cfg.visualization.showFigures
    if cfg.visualization.animate
        results.figure = animate_mission(results,cfg,mission,aoi);
    else
        results.figure = plot_mission_summary(results,cfg,mission,aoi);
    end
end
end

function data = signal_data(simOut,name)
ts = simOut.get(name);
data = squeeze(ts.Data);
if isvector(data)
    data = data(:);
elseif size(data,1) <= 3 && size(data,2) > size(data,1)
    data = data.';
end
end

function time = signal_time(simOut,name)
ts = simOut.get(name);
time = ts.Time(:);
end

function print_check_summary(c,cfg)
fprintf('\nVerification summary\n');
fprintf('  Search waypoints inside AOI:       %s\n',pass_text(c.searchWaypointsInsideAOI));
fprintf('  All flight phases visited:         %s  %s\n',pass_text(c.allFlightPhasesVisited),mat2str(c.phaseCodesVisited));
fprintf('  Mission complete / returned home:  %s / %s (error %.3f m)\n', ...
    pass_text(c.missionComplete),pass_text(c.returnedNearHome),c.returnErrorM);
fprintf('  Start / end altitude:              %.4f / %.4f m %s\n', ...
    c.startAltitudeM,c.endAltitudeM,pass_text(c.startsAndEndsOnGround));
fprintf('  Maximum altitude:                  %.3f m (limit %.2f m) %s\n', ...
    c.maxAltitudeM,cfg.requirements.maxAltitudeAGLM,pass_text(c.altitudeWithinRequirement));
fprintf('  Maximum simulated 3-D speed:       %.3f m/s (limit %.2f m/s) %s\n', ...
    c.maxSpeedMps,cfg.requirements.maxSpeedMps,pass_text(c.speedWithinRequirement));
fprintf('  Phase speed limits (actual):       %s %s m/s\n', ...
    pass_text(c.phaseSpeedLimitsMet),mat2str(c.maxActualSpeedByPhaseMps,3));
fprintf('  Mission duration:                  %.1f s (endurance %.0f s) %s\n', ...
    c.durationSec,cfg.requirements.minimumEnduranceSec,pass_text(c.withinEnduranceRequirement));
fprintf('  Plot limits contain full mission:  %s\n', ...
    pass_text(c.plotLimitsContainMission));
fprintf('  Overall automated checks:          %s\n\n',pass_text(c.allPassed));
end

function text = pass_text(value)
if value
    text = 'PASS';
else
    text = 'FAIL';
end
end
