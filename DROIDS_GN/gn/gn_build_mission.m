function [searchW, homeAir, entryPoint, lastPoint, mission, plan] = gn_build_mission(cfg)
%GN_BUILD_MISSION  Boustrophedon search + smoothed stage trajectories from gn_init cfg.
%   AOI: cfg.aoiPoly (2xN [N;E]) if present, else the cfg.aoiCorner/cfg.aoiSize rectangle.
%   The lanes are planned over the AOI grown by cfg.aoiBuffer (m) on every side.
%   plan.aoi_ne_m is the AOI itself, plan.search_ne_m the buffered area that was searched.
%   searchW Nx3 [N E D] search waypoints | homeAir, entryPoint, lastPoint 3x1 NED
%   mission: per-stage trajectories (mission_eval / stage_reference) | plan: raw planner output

    if isfield(cfg, 'aoiPoly')
        aoi = cfg.aoiPoly;
    else
        aoi = cfg.aoiCorner + [0, cfg.aoiSize(1), cfg.aoiSize(1), 0; 0, 0, cfg.aoiSize(2), cfg.aoiSize(2)];
    end
    search_poly = aoi;                                       % area the lanes are planned over
    if isfield(cfg, 'aoiBuffer') && cfg.aoiBuffer > 0
        search_poly = buffer_aoi(aoi, cfg.aoiBuffer);        % AOI grown by cfg.aoiBuffer on every side
    end
    heading = NaN;
    if isfield(cfg, 'laneHeading')
        heading = cfg.laneHeading;
    end
    fov_eq = 2*atan(cfg.laneSpacing / (2*cfg.altitude));    % FOV whose footprint equals cfg.laneSpacing

    lane_ext = 0;
    if isfield(cfg, 'laneExt')
        lane_ext = cfg.laneExt;
    end
    [mission, info, wp, typ, n] = plan_mission(search_poly, fov_eq, cfg.altitude, heading, cfg.home(1:2), cfg.vNom, cfg.aNom, lane_ext);
    if info.status ~= 0
        error('gn_build_mission:plan', 'search_boustrophedon status %d', info.status);
    end
    searchW = [wp(:, 2:n); -cfg.altitude*ones(1, n-1)]';
    homeAir = mission.p_end(:, 1);
    entryPoint = mission.p_end(:, 2);
    lastPoint = mission.p_end(:, 3);
    plan = struct('aoi_ne_m', aoi, 'search_ne_m', search_poly, 'wp_ne_m', wp, 'wp_type', typ, 'n_wp', n, 'info', info);
end
