function [velocityCommandENU, targetENU, waypointIndex, phaseCode, missionComplete] = ...
    guidance_step(estimatedPositionENU, truePositionENU, routeENU, ...
    phaseByWaypoint, routeCount, params)
%GUIDANCE_STEP Stateful waypoint switching and phase-aware velocity guidance.
%#codegen
% params = [acceptanceRadius, transitSpeed, searchSpeed, climbSpeed,
%           descentSpeed, slowdownRadius, landingHorizontalTolerance,
%           landingAltitudeTolerance]

persistent activeWaypoint
if isempty(activeWaypoint)
    activeWaypoint = 1;
end

acceptanceRadius = params(1);
slowdownRadius = params(6);

% Skip any waypoint already inside the acceptance sphere. The while loop
% also handles consecutive or repeated waypoints without a one-step delay.
while activeWaypoint <= routeCount
    candidate = routeENU(activeWaypoint,:).';
    candidatePhase = phaseByWaypoint(activeWaypoint);
    if candidatePhase == 5
        % Landing completion uses true plant position and a ground-contact
        % threshold. Normal noisy waypoint acceptance could otherwise stop
        % the simulation while the vehicle is still above the ground.
        horizontalError = norm(candidate(1:2) - truePositionENU(1:2));
        reached = horizontalError <= params(7) && ...
            truePositionENU(3) <= params(8);
    else
        reached = norm(candidate - estimatedPositionENU) < acceptanceRadius;
    end
    if ~reached
        break;
    end
    activeWaypoint = activeWaypoint + 1;
end

if activeWaypoint > routeCount
    velocityCommandENU = zeros(3,1);
    targetENU = routeENU(routeCount,:).';
    waypointIndex = routeCount;
    phaseCode = 6;
    missionComplete = true;
    return;
end

targetENU = routeENU(activeWaypoint,:).';
waypointIndex = activeWaypoint;
phaseCode = phaseByWaypoint(activeWaypoint);
missionComplete = false;

delta = targetENU - estimatedPositionENU;
distance = norm(delta);
switch phaseCode
    case 1
        speedLimit = params(4);
    case 2
        speedLimit = params(2);
    case 3
        speedLimit = params(3);
    case 4
        speedLimit = params(2);
    case 5
        speedLimit = params(5);
    otherwise
        speedLimit = 0;
end

if distance > 1e-9
    % Linear slowdown near each waypoint improves switching behavior and
    % makes a first-order point-mass model settle without overshoot.
    speed = speedLimit * min(1, distance/slowdownRadius);
    velocityCommandENU = speed * delta/distance;
else
    velocityCommandENU = zeros(3,1);
end
end
