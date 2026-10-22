function [S,homeAir,entryPoint,lastPoint] = gn_buildSearch(cfg)
% Generate a serpentine search path in local NED coordinates.

% Read the AOI corner and dimensions.
n0 = cfg.aoiCorner(1);
e0 = cfg.aoiCorner(2);
Ln = cfg.aoiSize(1);
Le = cfg.aoiSize(2);

% Include both AOI edges while limiting the lane spacing.
nIntervals = ceil(Le/cfg.laneSpacing);
eastLanes = linspace(e0,e0+Le,nIntervals+1);

% Each lane contributes two endpoints.
% Columns represent North, East, and Down.
S = zeros(2*numel(eastLanes),3);

for j = 1:numel(eastLanes)
    % Reverse alternate lanes to create a serpentine path.
    if mod(j,2) == 1
        ns = [n0; n0+Ln];
    else
        ns = [n0+Ln; n0];
    end

    rows = (2*j-1):(2*j);

    % Assign columns separately to avoid concatenation ambiguity.
    S(rows,1) = ns;
    S(rows,2) = eastLanes(j);
    S(rows,3) = -cfg.altitude;
end

% Return column vectors for the mission reference positions.
homeAir = [cfg.home(1); cfg.home(2); -cfg.altitude];
entryPoint = S(1,:).';
lastPoint = S(end,:).';

% Verify the altitude limit and reject duplicate adjacent points.
assert(all(-S(:,3) <= cfg.hMax));
assert(all(vecnorm(diff(S),2,2) > 0));
end