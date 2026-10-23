% Mission run: gn_init config -> boustrophedon search -> smoothed flight stages -> plots.
clear; clc; close all;

random_aoi = true;      % true: random polygon in a 30x30 m area; false: gn_init AOI rectangle
run_simulink = false;   % true: also run gncc_uav_toolbox and overlay the flown path
run_power = true;      % true: also run power/main_motorf_robust on this mission (motor forces + power draw)
area_m = 30;
min_size_m = 5;
start_dist_m = [10 20];
z_exag = 4;

addpath(fullfile(fileparts(mfilename('fullpath')), 'gn'));
gn_init;
height_m = cfg.altitude;
v_max_mps = cfg.vNom;
a_max_mps2 = cfg.aNom;
dt_s = cfg.ctrlTs;
start_ne_m = cfg.home(1:2);
area_lo = [0; 0];

if random_aoi
    nv = randi([4 12]);
    ang = (0:nv-1)/nv*2*pi + rand(1, nv)*2*pi/nv*0.8;
    r = 0.35 + 0.65*rand(1, nv);
    xy = [r.*cos(ang); r.*sin(ang)];
    xy = xy * min_size_m / min(max(xy, [], 2) - min(xy, [], 2)) * (1 + 0.5*rand);
    xy = xy - min(xy, [], 2);
    poly_ne_m = xy + rand(2, 1) .* (area_m - max(xy, [], 2));

    while true
        start_ne_m = area_m * rand(2, 1);
        d = poly_dist(start_ne_m, poly_ne_m);
        if d >= start_dist_m(1) && d <= start_dist_m(2) && ...
                ~inpolygon(start_ne_m(1), start_ne_m(2), poly_ne_m(1, :), poly_ne_m(2, :))
            break
        end
    end
    area_lo = cfg.home(1:2) - start_ne_m;
    cfg.aoiPoly = poly_ne_m + area_lo;
    start_ne_m = cfg.home(1:2);
end

tic;
[searchW, homeAir, entryPoint, lastPoint, mission, plan] = gn_build_mission(cfg);
t_ms = 1e3*toc;
poly_ne_m = plan.aoi_ne_m;
wp = plan.wp_ne_m;
typ = plan.wp_type;
n = plan.n_wp;
info = plan.info;
d = poly_dist(start_ne_m, poly_ne_m);
if ~random_aoi
    area_lo = floor(min([poly_ne_m, start_ne_m], [], 2)) - 2;
    area_m = ceil(max(max([poly_ne_m, start_ne_m], [], 2) - area_lo)) + 2;
end
wp_ned_m = [[start_ne_m; 0], [start_ne_m; -height_m], [wp(:, 2:n); -height_m*ones(1, n-1)], ...
    [start_ne_m; -height_m], [start_ne_m; 0]];

clear mission_sequencer
K = ceil((sum(mission.T) + 10) / dt_s);
t_s = (0:K-1) * dt_s;
p = zeros(3, K); v = p; a = p;
stg = zeros(1, K);
p_meas = mission.home_ned_m;
for i = 1:K
    [stg(i), t_in] = mission_sequencer(mission, t_s(i), true, p_meas);
    [p(:, i), v(:, i), a(:, i)] = mission_eval(mission, stg(i), t_in);
    p_meas = p(:, i);
    if stg(i) == 6
        break
    end
end
K = i;
t_s = t_s(1:K); p = p(:, 1:K); v = v(:, 1:K); a = a(:, 1:K); stg = stg(1:K);
T_end = t_s(end);
t_stage = [arrayfun(@(s) t_s(find(stg == s, 1)), 1:5), T_end];
fprintf(['status %d | %d lanes | %d cells | heading %.1f deg | start %.2f m from area | %d transits outside\n' ...
    'mission %.1f s | path %.1f m | max |v| %.3f m/s | max |a| %.3f m/s^2 | plan %.1f ms\n'], ...
    info.status, info.n_lanes, info.n_cells, rad2deg(info.lane_heading_rad), d, info.n_transit_outside, ...
    T_end, sum(vecnorm(diff(p, 1, 2))), max(vecnorm(v)), max(vecnorm(a)), t_ms);

wp_plan = [wp(:, 1:n), start_ne_m];
typ_plan = [typ(1:n), 4];
fig0 = figure(); hold on; grid on;
fig0.Theme = 'light';
plot3(area_lo(2) + [0 area_m area_m 0 0], area_lo(1) + [0 0 area_m area_m 0], zeros(1, 5), 'k-', 'LineWidth', 1, 'DisplayName', 'test area');
fill3(poly_ne_m(2, :), poly_ne_m(1, :), zeros(1, size(poly_ne_m, 2)), [0.85 0.9 1], 'EdgeColor', 'k', 'DisplayName', 'search area');
plot3(start_ne_m([2 2]), start_ne_m([1 1]), [0 height_m], 'k:', 'LineWidth', 1.5, 'DisplayName', 'takeoff / landing');
names = {'transit', 'lane', 'connector', 'return'};
styles = {'r--', 'b-', 'g-', 'm--'};
for k = 1:4
    first = true;
    for i = 2:numel(typ_plan)
        if typ_plan(i) == k
            h = plot3(wp_plan(2, i-1:i), wp_plan(1, i-1:i), [height_m height_m], styles{k}, 'LineWidth', 1.5);
            if first
                h.DisplayName = names{k};
                first = false;
            else
                h.HandleVisibility = 'off';
            end
        end
    end
end
plot3(start_ne_m(2), start_ne_m(1), 0, 'ko', 'MarkerFaceColor', 'y', 'DisplayName', 'start / home');
plot3(wp_plan(2, :), wp_plan(1, :), zeros(1, numel(typ_plan)), '-', 'Color', [0.6 0.6 0.6], 'DisplayName', 'ground track');
xlabel('East [m]'); ylabel('North [m]'); zlabel('Altitude AGL [m]');
xlim(area_lo(2) + [0 area_m]); ylim(area_lo(1) + [0 area_m]); zlim([0 1.5*height_m]);
daspect([1 1 1/z_exag]); view(3);
title({'Waypoint plan (unsmoothed)', sprintf('3D (E, N, up), z x%d | FOV %.0f deg, %.0f%% overlap, h %.1f m, spacing %.2f m', ...
    z_exag, cfg.cameraCrossTrackFOV, 100*cfg.cameraLaneOverlap, height_m, cfg.laneSpacing)});
legend('Location', 'bestoutside');

fig = figure(); hold on; grid on;
fig.Theme = 'light';
plot3(area_lo(2) + [0 area_m area_m 0 0], area_lo(1) + [0 0 area_m area_m 0], zeros(1, 5), 'k-', 'LineWidth', 1, 'DisplayName', 'test area');
fill3(poly_ne_m(2, :), poly_ne_m(1, :), zeros(1, size(poly_ne_m, 2)), [0.85 0.9 1], 'EdgeColor', 'k', 'DisplayName', 'search area');
plot3(wp_ned_m(2, :), wp_ned_m(1, :), -wp_ned_m(3, :), '--', 'Color', [0.6 0.6 0.6], 'DisplayName', 'waypoints');
plot3(p(2, :), p(1, :), -p(3, :), 'b-', 'LineWidth', 1.5, 'DisplayName', 'smoothed trajectory');
plot3(p(2, :), p(1, :), zeros(1, K), '-', 'Color', [0.6 0.6 0.6], 'DisplayName', 'ground track');
plot3(start_ne_m(2), start_ne_m(1), 0, 'ko', 'MarkerFaceColor', 'y', 'DisplayName', 'start / home');
xlabel('East [m]'); ylabel('North [m]'); zlabel('Altitude AGL [m]');
xlim(area_lo(2) + [0 area_m]); ylim(area_lo(1) + [0 area_m]); zlim([0 1.5*height_m]);
daspect([1 1 1/z_exag]); view(3);
title(sprintf('Mission, 3D (E, N, up), z x%d | FOV %.0f deg, %.0f%% overlap, h %.1f m, spacing %.2f m', ...
    z_exag, cfg.cameraCrossTrackFOV, 100*cfg.cameraLaneOverlap, height_m, cfg.laneSpacing));
legend('Location', 'bestoutside');

stage_names = {'takeoff', 'transit', 'search', 'return', 'landing'};
stage_colors = [0.5 0.5 0.5; 1 0 0; 0 0 1; 1 0 1; 0.2 0.2 0.2];
fig2 = figure();
fig2.Theme = 'light';
for sp = 1:2
    subplot(2, 1, sp); hold on; grid on;
    for j = 1:5
        xregion(t_stage(j), t_stage(j+1), 'FaceColor', stage_colors(j, :), 'FaceAlpha', 0.08, ...
            'DisplayName', stage_names{j});
        lbl = '';
        if sp == 1
            lbl = stage_names{j};
        end
        xline(t_stage(j), ':', lbl, 'Color', stage_colors(j, :), 'LabelVerticalAlignment', 'bottom', ...
            'LabelOrientation', 'aligned', 'HandleVisibility', 'off');
    end
    if sp == 1
        plot(t_s, vecnorm(v), 'b-', 'LineWidth', 1.2, 'DisplayName', '|v|');
        yline(v_max_mps, 'r--', 'DisplayName', 'v_{nom}');
        ylabel('Speed [m/s]'); ylim([0 1.2*v_max_mps]);
        title('Reference speed and acceleration by flight stage (NED)');
    else
        plot(t_s, vecnorm(a), 'b-', 'LineWidth', 1.2, 'DisplayName', '|a|');
        yline(a_max_mps2, 'r--', 'DisplayName', 'a_{nom}');
        xlabel('Time [s]'); ylabel('Acceleration [m/s^2]'); ylim([0 1.2*a_max_mps2]);
    end
    xlim([0 T_end]);
    legend('Location', 'eastoutside');
end

axis_names = {'x (N)', 'y (E)', 'z (D)'};
axis_colors = [0 0.45 0.74; 0.85 0.33 0.1; 0.47 0.67 0.19];
fig3 = figure();
fig3.Theme = 'light';
for sp = 1:2
    subplot(2, 1, sp); hold on; grid on;
    for j = 1:5
        xregion(t_stage(j), t_stage(j+1), 'FaceColor', stage_colors(j, :), 'FaceAlpha', 0.08, ...
            'HandleVisibility', 'off');
        xline(t_stage(j), ':', 'Color', stage_colors(j, :), 'HandleVisibility', 'off');
    end
    for c = 1:3
        if sp == 1
            plot(t_s, v(c, :), '-', 'Color', axis_colors(c, :), 'LineWidth', 1.2, 'DisplayName', ['v_' axis_names{c}]);
        else
            plot(t_s, a(c, :), '-', 'Color', axis_colors(c, :), 'LineWidth', 1.2, 'DisplayName', ['a_' axis_names{c}]);
        end
    end
    if sp == 1
        yline([-v_max_mps v_max_mps], 'r--', 'HandleVisibility', 'off');
        ylabel('Velocity [m/s]'); ylim(1.2*[-v_max_mps v_max_mps]);
        title('Reference velocity and acceleration components (NED)');
    else
        yline([-a_max_mps2 a_max_mps2], 'r--', 'HandleVisibility', 'off');
        xlabel('Time [s]'); ylabel('Acceleration [m/s^2]'); ylim(1.2*[-a_max_mps2 a_max_mps2]);
    end
    xlim([0 T_end]);
    legend('Location', 'eastoutside');
end

if run_simulink
    out = sim('gncc_uav_toolbox', 'StopTime', num2str(T_end + 60));
    ph = out.phaseLog;
    t_done = ph.Time(find(squeeze(ph.Data) == 6, 1));
    fprintf('Simulink: complete (phase 6) at %s s | reference mission %.1f s\n', num2str(t_done), T_end);
    if any(strcmp(out.who, 'posLog'))
        ps = squeeze(out.posLog.Data);
        if size(ps, 1) ~= 3
            ps = ps.';
        end
        figure(fig);
        plot3(ps(2, :), ps(1, :), -ps(3, :), 'r-', 'LineWidth', 1.2, 'DisplayName', 'Simulink flown');
    end
end

if run_power
    power_results = main_motorf_robust(cfg);
end

% Shortest distance from point p to the polygon boundary.
function d = poly_dist(p, poly)
    A = poly;
    B = poly(:, [2:end 1]);
    AB = B - A;
    s = max(0, min(1, sum((p - A).*AB, 1) ./ sum(AB.^2, 1)));
    d = min(vecnorm(p - (A + AB.*s)));
end
