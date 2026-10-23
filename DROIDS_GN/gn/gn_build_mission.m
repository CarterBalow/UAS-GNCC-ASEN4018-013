function [searchW, homeAir, entryPoint, lastPoint, mission, plan] = gn_build_mission(cfg)
%GN_BUILD_MISSION  Boustrophedon search + smoothed stage trajectories from gn_init cfg.
%   AOI: cfg.aoiPoly (2xN [N;E]) if present, else the cfg.aoiCorner/cfg.aoiSize rectangle.
%   searchW Nx3 [N E D] search waypoints | homeAir, entryPoint, lastPoint 3x1 NED
%   mission: per-stage trajectories (mission_eval / stage_reference) | plan: raw planner output

    if isfield(cfg, 'aoiPoly')
        aoi = cfg.aoiPoly;
    else
        aoi = cfg.aoiCorner + [0, cfg.aoiSize(1), cfg.aoiSize(1), 0; 0, 0, cfg.aoiSize(2), cfg.aoiSize(2)];
    end
    heading = NaN;
    if isfield(cfg, 'laneHeading')
        heading = cfg.laneHeading;
    end
    fov_eq = 2*atan(cfg.laneSpacing / (2*cfg.altitude));    % FOV whose footprint equals cfg.laneSpacing

    [mission, info, wp, typ, n] = plan_mission(aoi, fov_eq, cfg.altitude, heading, cfg.home(1:2), cfg.vNom, cfg.aNom);
    if info.status ~= 0
        error('gn_build_mission:plan', 'search_boustrophedon status %d', info.status);
    end
    searchW = [wp(:, 2:n); -cfg.altitude*ones(1, n-1)]';
    homeAir = mission.p_end(:, 1);
    entryPoint = mission.p_end(:, 2);
    lastPoint = mission.p_end(:, 3);
    plan = struct('aoi_ne_m', aoi, 'wp_ne_m', wp, 'wp_type', typ, 'n_wp', n, 'info', info);
end
