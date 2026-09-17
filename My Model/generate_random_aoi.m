function aoi = generate_random_aoi(cfg)
%GENERATE_RANDOM_AOI Make an axis-aligned rectangular AOI in local ENU.
% The AOI is randomized on every normal run. Set cfg.randomSeed to an
% integer for a reproducible scenario.

if isempty(cfg.randomSeed)
    rng('shuffle');
else
    rng(cfg.randomSeed, 'twister');
end

distanceM = uniform_sample(cfg.aoi.centerDistanceRangeM);
bearingDeg = uniform_sample(cfg.aoi.bearingRangeDeg);
widthM = uniform_sample(cfg.aoi.sizeRangeM);
heightM = uniform_sample(cfg.aoi.sizeRangeM);

centerEN = cfg.homeENU(1:2) + distanceM * ...
    [cosd(bearingDeg), sind(bearingDeg)];

aoi.centerEN = centerEN;
aoi.widthM = widthM;
aoi.heightM = heightM;
aoi.bearingFromEastDeg = bearingDeg;
aoi.centerDistanceFromHomeM = distanceM;
aoi.xMin = centerEN(1) - widthM/2;
aoi.xMax = centerEN(1) + widthM/2;
aoi.yMin = centerEN(2) - heightM/2;
aoi.yMax = centerEN(2) + heightM/2;
aoi.boundaryEN = [aoi.xMin, aoi.yMin; ...
                  aoi.xMax, aoi.yMin; ...
                  aoi.xMax, aoi.yMax; ...
                  aoi.xMin, aoi.yMax; ...
                  aoi.xMin, aoi.yMin];
end

function value = uniform_sample(bounds)
value = bounds(1) + (bounds(2)-bounds(1))*rand();
end
