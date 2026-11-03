function [mission, info, wp_ne_m, wp_type, n_wp] = plan_mission(poly_ne_m, fov_rad, height_m, lane_heading_rad, home_ne_m, v_max_mps, a_max_mps2) %#codegen
%PLAN_MISSION  One trajectory per flight stage: takeoff, transit, search, return, landing (local NED).
%   Each stage starts and ends at rest. Evaluate with mission_eval; sequence with mission_sequencer.
%   mission.T(s) stage durations | mission.p_end(:, s) stage end points | info from search_boustrophedon

    [wp_ne_m, wp_type, n_wp, info] = search_boustrophedon(poly_ne_m, fov_rad, height_m, lane_heading_rad, home_ne_m);
    psi = info.lane_heading_rad;
    if ~isfinite(psi)
        psi = 0;
    end
    M = size(wp_ne_m, 2);
    ground = [home_ne_m; 0];
    above = [home_ne_m; -height_m];

    srch = zeros(3, M);
    ns = 0;
    if info.status == 0
        for i = 2:n_wp
            ns = ns + 1;
            srch(:, ns) = [wp_ne_m(:, i); -height_m];
        end
    end
    if ns == 0
        srch(:, 1) = above;
        ns = 1;
    end
    entry = srch(:, 1);
    leave = srch(:, ns);

    no_stop2 = false(1, 2);
    mission.takeoff     = smooth_path([ground, above], 2, no_stop2, v_max_mps, a_max_mps2, psi);
    mission.transit     = smooth_path([above, entry], 2, no_stop2, v_max_mps, a_max_mps2, psi);
    mission.search      = smooth_path(srch, ns, false(1, M), v_max_mps, a_max_mps2, psi);
    mission.return_home = smooth_path([leave, above], 2, no_stop2, v_max_mps, a_max_mps2, psi);
    mission.landing     = smooth_path([above, ground], 2, no_stop2, v_max_mps, a_max_mps2, psi);
    mission.home_ned_m = ground;
    mission.psi = psi;
    mission.T = [mission.takeoff.T, mission.transit.T, mission.search.T, mission.return_home.T, mission.landing.T];
    mission.p_end = [above, entry, leave, above, ground];
end
