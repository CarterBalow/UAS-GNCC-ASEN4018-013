function mission = generate_mission_waypoints(cfg, aoi)
%GENERATE_MISSION_WAYPOINTS Build takeoff, transit, serpentine, return, land.
% All rows use [East, North, Up] in meters.

numberOfTrackIntervals = ceil(aoi.heightM / cfg.aoi.trackSpacingM);
trackNorth = linspace(aoi.yMin, aoi.yMax, numberOfTrackIntervals + 1);

% Try both possible first-track directions and choose the closer entry.
candidateWest = [aoi.xMin, trackNorth(1)];
candidateEast = [aoi.xMax, trackNorth(1)];
startAtWest = norm(candidateWest - cfg.homeENU(1:2)) <= ...
              norm(candidateEast - cfg.homeENU(1:2));

searchEN = zeros(2*numel(trackNorth), 2);
row = 1;
for k = 1:numel(trackNorth)
    westToEast = xor(mod(k,2) == 0, startAtWest);
    if westToEast
        endpoints = [aoi.xMin, aoi.xMax];
    else
        endpoints = [aoi.xMax, aoi.xMin];
    end
    searchEN(row,:) = [endpoints(1), trackNorth(k)];
    searchEN(row+1,:) = [endpoints(2), trackNorth(k)];
    row = row + 2;
end

searchENU = [searchEN, cfg.flight.searchAltitudeM*ones(size(searchEN,1),1)];
takeoff = [cfg.homeENU(1:2), cfg.flight.transitAltitudeM];
homeOverhead = [cfg.homeENU(1:2), cfg.flight.transitAltitudeM];
homeGround = cfg.homeENU;

% The first AOI point is the transit target. Subsequent AOI points are the
% search legs. This avoids duplicating a waypoint while preserving phases.
waypointsENU = [takeoff; searchENU; homeOverhead; homeGround];
phaseByWaypoint = [cfg.phase.takeoff; cfg.phase.transit; ...
    cfg.phase.search*ones(size(searchENU,1)-1,1); ...
    cfg.phase.return; cfg.phase.landing];

assert(size(waypointsENU,1) <= cfg.simulation.maxWaypoints, ...
    'Generated route exceeds cfg.simulation.maxWaypoints.');

tolerance = 10*eps(max(abs([aoi.boundaryEN(:); 1])));
inside = searchENU(:,1) >= aoi.xMin-tolerance & ...
         searchENU(:,1) <= aoi.xMax+tolerance & ...
         searchENU(:,2) >= aoi.yMin-tolerance & ...
         searchENU(:,2) <= aoi.yMax+tolerance;
assert(all(inside), 'A generated search waypoint lies outside the AOI.');

mission.waypointsENU = waypointsENU;
mission.phaseByWaypoint = phaseByWaypoint;
mission.searchWaypointsENU = searchENU;
mission.searchWaypointInsideAOI = inside;
mission.actualTrackSpacingM = aoi.heightM / numberOfTrackIntervals;
mission.routeCount = size(waypointsENU,1);
mission.phaseNames = cfg.phase.names;
end
