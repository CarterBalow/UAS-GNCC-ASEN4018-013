function cfg = mission_config(overrides)
%MISSION_CONFIG Single source for mission, vehicle, and visualization values.
% Coordinate order is [East, North, Up] in meters (ENU). Altitude is Up/AGL.
%
% Values tagged REQUIREMENT come from the project RFP. Values tagged
% ASSUMPTION are editable engineering choices for this simulation.

if nargin < 1
    overrides = struct();
end

cfg.project.coordinateFrame = 'ENU';
cfg.project.coordinateOrder = '[East, North, Up]';
cfg.project.lengthUnits = 'm';
cfg.project.speedUnits = 'm/s';
cfg.project.sourceDocument = 'ASEN 4018-013_014 AY26-27 RFP v2.pdf';

% RFP requirements.
cfg.requirements.maxAltitudeAGLM = 2.0;       % REQUIREMENT: no higher than 2 m AGL
cfg.requirements.maxSpeedMps = 0.5;           % REQUIREMENT: no more than 0.5 m/s
cfg.requirements.minimumEnduranceSec = 15*60; % REQUIREMENT: at least 15 minutes
cfg.requirements.nominalAOISizeM = [5, 5];    % REQUIREMENT/definition: approximately 5 m x 5 m

% Mission geometry and randomization.
cfg.randomSeed = []; % ASSUMPTION: [] reshuffles; set an integer to reproduce a run
cfg.homeENU = [0, 0, 0];
cfg.aoi.centerDistanceRangeM = [10, 18]; % ASSUMPTION: distance from home to AOI center
cfg.aoi.bearingRangeDeg = [0, 360];      % ASSUMPTION: bearing clockwise from East
cfg.aoi.sizeRangeM = [4.5, 5.5];         % ASSUMPTION: independently sampled width/height
cfg.aoi.trackSpacingM = 1.0;             % ASSUMPTION: maximum spacing between search tracks

% Phase-dependent guidance settings. All stay below the global RFP limits.
cfg.flight.transitAltitudeM = 1.5;       % ASSUMPTION
cfg.flight.searchAltitudeM = 1.5;        % ASSUMPTION
cfg.flight.transitSpeedMps = 0.30;       % ASSUMPTION: physical transit speed
cfg.flight.searchSpeedMps = 0.20;        % ASSUMPTION: physical search speed
cfg.flight.climbSpeedMps = 0.20;         % ASSUMPTION: physical climb speed
cfg.flight.descentSpeedMps = 0.15;       % ASSUMPTION: physical descent speed
cfg.guidance.acceptanceRadiusM = 0.25;    % ASSUMPTION
cfg.guidance.slowdownRadiusM = 1.0;       % ASSUMPTION: reduces overshoot at waypoints
cfg.guidance.landingHorizontalToleranceM = 0.10; % ASSUMPTION: true-position landing check
cfg.guidance.landingAltitudeToleranceM = 0.0;    % Ground contact: Up must reach exactly zero

% Simplified vehicle and navigation model.
cfg.vehicle.velocityTimeConstantSec = 0.50; % ASSUMPTION: first-order turn/speed response
cfg.navigation.positionNoiseStdM = 0.02;    % ASSUMPTION: per-axis position noise
cfg.navigation.filterCoefficient = 0.20;    % ASSUMPTION: first-order discrete LPF coefficient

cfg.simulation.sampleTimeSec = 0.05;
cfg.simulation.maxTimeSec = 600;
cfg.simulation.maxWaypoints = 200;
cfg.simulation.modelName = 'multirotor_gn_model';

cfg.visualization.showFigures = true;
cfg.visualization.animate = true;
cfg.visualization.frameStride = 4;        % Plot every Nth simulation sample
cfg.visualization.playbackSpeed = 20;    % Animation only: simulated seconds / real second
cfg.visualization.horizontalPaddingFraction = 0.08;
cfg.visualization.minimumHorizontalPaddingM = 1.0;
cfg.visualization.verticalPaddingFraction = 0.12;
cfg.visualization.minimumVerticalPaddingM = 0.15;

cfg.verification.altitudeToleranceM = 0.01;
cfg.verification.speedToleranceMps = 0.01;
cfg.verification.returnToleranceM = 0.50;

cfg.phase.names = {'TAKEOFF','TRANSIT','SEARCH','RETURN','LANDING','COMPLETE'};
cfg.phase.takeoff = 1;
cfg.phase.transit = 2;
cfg.phase.search = 3;
cfg.phase.return = 4;
cfg.phase.landing = 5;
cfg.phase.complete = 6;

cfg = merge_structs(cfg, overrides);
validate_config(cfg);
end

function out = merge_structs(base, updates)
out = base;
if isempty(updates)
    return;
end
names = fieldnames(updates);
for k = 1:numel(names)
    name = names{k};
    if isfield(out, name) && isstruct(out.(name)) && isstruct(updates.(name))
        out.(name) = merge_structs(out.(name), updates.(name));
    else
        out.(name) = updates.(name);
    end
end
end

function validate_config(cfg)
assert(cfg.flight.transitAltitudeM <= cfg.requirements.maxAltitudeAGLM, ...
    'Transit altitude exceeds the RFP altitude limit.');
assert(cfg.flight.searchAltitudeM <= cfg.requirements.maxAltitudeAGLM, ...
    'Search altitude exceeds the RFP altitude limit.');
phaseSpeeds = [cfg.flight.transitSpeedMps, cfg.flight.searchSpeedMps, ...
    cfg.flight.climbSpeedMps, cfg.flight.descentSpeedMps];
assert(all(phaseSpeeds <= cfg.requirements.maxSpeedMps), ...
    'A configured phase speed exceeds the RFP speed limit.');
assert(cfg.aoi.trackSpacingM > 0, 'Track spacing must be positive.');
assert(cfg.guidance.acceptanceRadiusM > 0, 'Acceptance radius must be positive.');
assert(cfg.guidance.slowdownRadiusM > cfg.guidance.acceptanceRadiusM, ...
    'Slowdown radius must be larger than the acceptance radius.');
assert(cfg.guidance.landingHorizontalToleranceM > 0, ...
    'Landing horizontal tolerance must be positive.');
assert(cfg.guidance.landingAltitudeToleranceM >= 0, ...
    'Landing altitude tolerance cannot be negative.');
assert(cfg.visualization.playbackSpeed > 0, ...
    'Animation playback speed must be positive.');
assert(cfg.simulation.maxWaypoints >= 10, 'maxWaypoints is too small for the mission.');
end
