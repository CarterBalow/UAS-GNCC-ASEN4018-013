function fig = animate_mission(results, cfg, mission, aoi)
%ANIMATE_MISSION Animate the 3-D ENU flight with supporting compliance views.

fig = figure('Name','Multirotor Guidance and Navigation Mission', ...
    'Color','w','NumberTitle','off','Position',[100 100 1400 800]);
layout = tiledlayout(fig,3,3,'TileSpacing','compact','Padding','compact');
plotLimits = results.plotLimits;

ax3D = nexttile(layout,[3 2]);
hold(ax3D,'on'); grid(ax3D,'on'); axis(ax3D,'equal');
style_axes(ax3D);
searchAltitude = cfg.flight.searchAltitudeM;
patch(ax3D,aoi.boundaryEN(:,1),aoi.boundaryEN(:,2),zeros(size(aoi.boundaryEN,1),1), ...
    [0.78 0.86 0.96], 'EdgeColor',[0.1 0.35 0.75], 'LineWidth',1.5, ...
    'FaceAlpha',0.40,'DisplayName','AOI ground footprint');
patch(ax3D,aoi.boundaryEN(:,1),aoi.boundaryEN(:,2), ...
    searchAltitude*ones(size(aoi.boundaryEN,1),1),[0.65 0.90 0.95], ...
    'EdgeColor',[0.1 0.55 0.65], 'LineStyle',':', 'LineWidth',1.2, ...
    'FaceAlpha',0.10,'DisplayName','Search-altitude plane');
for corner = 1:4
    plot3(ax3D,[aoi.boundaryEN(corner,1) aoi.boundaryEN(corner,1)], ...
        [aoi.boundaryEN(corner,2) aoi.boundaryEN(corner,2)], ...
        [0 searchAltitude],':','Color',[0.1 0.55 0.65], ...
        'HandleVisibility','off');
end
plot3(ax3D,mission.waypointsENU(:,1),mission.waypointsENU(:,2), ...
    mission.waypointsENU(:,3),'--o','Color',[0.30 0.30 0.30], ...
    'MarkerSize',3,'DisplayName','Planned 3-D route');
plot3(ax3D,cfg.homeENU(1),cfg.homeENU(2),cfg.homeENU(3),'p','MarkerSize',13, ...
    'MarkerFaceColor',[0.2 0.65 0.2], 'DisplayName','Home');
traveled3D = animatedline(ax3D,'Color',[0.85 0.15 0.15], ...
    'LineWidth',1.8,'DisplayName','Traveled path');
vehicleMarker3D = plot3(ax3D,nan,nan,nan,'o','MarkerSize',9, ...
    'MarkerFaceColor',[0.9 0.1 0.1], 'MarkerEdgeColor','w', ...
    'DisplayName','Multirotor');
targetMarker3D = plot3(ax3D,nan,nan,nan,'x','MarkerSize',11,'LineWidth',2, ...
    'Color',[0.75 0 0.75],'DisplayName','Current target');
xlabel(ax3D,'East (m)'); ylabel(ax3D,'North (m)'); zlabel(ax3D,'Up / altitude AGL (m)');
view(ax3D,38,24);
daspect(ax3D,[1 1 0.35]); % Deliberate vertical exaggeration for visibility.
xlim(ax3D,plotLimits.eastM);
ylim(ax3D,plotLimits.northM);
zlim(ax3D,plotLimits.upM);
mapLegend = legend(ax3D,'Location','best');
set(mapLegend,'Color','w','TextColor','k','EdgeColor',[0.4 0.4 0.4]);

axTop = nexttile(layout);
hold(axTop,'on'); grid(axTop,'on'); axis(axTop,'equal');
style_axes(axTop);
aoiTop = patch(axTop,aoi.boundaryEN(:,1),aoi.boundaryEN(:,2),[0.88 0.94 1.0], ...
    'EdgeColor',[0.1 0.35 0.75],'FaceAlpha',0.4, ...
    'DisplayName','AOI boundary');
plannedTop = plot(axTop,mission.waypointsENU(:,1),mission.waypointsENU(:,2),'--', ...
    'Color',[0.4 0.4 0.4],'DisplayName','Planned route (dashed)');
traveledTop = animatedline(axTop,'Color',[0.85 0.15 0.15],'LineWidth',1.3, ...
    'DisplayName','Simulated trajectory');
vehicleMarkerTop = plot(axTop,nan,nan,'o','MarkerSize',6, ...
    'MarkerFaceColor',[0.9 0.1 0.1],'MarkerEdgeColor','w');
targetMarkerTop = plot(axTop,nan,nan,'x','MarkerSize',8,'LineWidth',1.5, ...
    'Color',[0.75 0 0.75]);
xlabel(axTop,'East (m)'); ylabel(axTop,'North (m)');
title(axTop,'Top-down navigation view','Color','k');
xlim(axTop,plotLimits.eastM);
ylim(axTop,plotLimits.northM);
topLegend = legend(axTop,[aoiTop plannedTop traveledTop],'Location','best');
set(topLegend,'Color','w','TextColor','k','EdgeColor',[0.4 0.4 0.4]);

axAlt = nexttile(layout);
hold(axAlt,'on');
style_axes(axAlt);
altitudeTrace = plot(axAlt,results.timeSec,results.positionENU(:,3),'b', ...
    'LineWidth',1.3,'DisplayName','Simulated altitude');
grid(axAlt,'on');
altitudeLimit = yline(axAlt,cfg.requirements.maxAltitudeAGLM,'r--', ...
    'LineWidth',1.2,'DisplayName',sprintf('RFP limit: %.1f m', ...
    cfg.requirements.maxAltitudeAGLM));
xlabel(axAlt,'Time (s)'); ylabel(axAlt,'Up / altitude AGL (m)');
title(axAlt,'Altitude compliance','Color','k');
altitudeLegend = legend(axAlt,[altitudeTrace altitudeLimit],'Location','best');
set(altitudeLegend,'Color','w','TextColor','k','EdgeColor',[0.4 0.4 0.4]);

axSpeed = nexttile(layout);
hold(axSpeed,'on');
style_axes(axSpeed);
speed = vecnorm(results.velocityENU,2,2);
speedTrace = plot(axSpeed,results.timeSec,speed,'Color',[0.1 0.55 0.2], ...
    'LineWidth',1.3,'DisplayName','Simulated 3-D speed');
grid(axSpeed,'on');
speedLimit = yline(axSpeed,cfg.requirements.maxSpeedMps,'r--', ...
    'LineWidth',1.2,'DisplayName',sprintf('RFP limit: %.1f m/s', ...
    cfg.requirements.maxSpeedMps));
xlabel(axSpeed,'Time (s)'); ylabel(axSpeed,'3-D speed (m/s)');
title(axSpeed,'Speed compliance','Color','k');
speedLegend = legend(axSpeed,[speedTrace speedLimit],'Location','best');
set(speedLegend,'Color','w','TextColor','k','EdgeColor',[0.4 0.4 0.4]);

if ~cfg.visualization.animate
    addpoints(traveled3D,results.positionENU(:,1),results.positionENU(:,2), ...
        results.positionENU(:,3));
    addpoints(traveledTop,results.positionENU(:,1),results.positionENU(:,2));
    k = numel(results.timeSec);
    update_markers_and_title(k);
else
    indices = 1:cfg.visualization.frameStride:numel(results.timeSec);
    if indices(end) ~= numel(results.timeSec)
        indices(end+1) = numel(results.timeSec);
    end
    previousIndex = indices(1);
    for k = indices
        addpoints(traveled3D,results.positionENU(k,1),results.positionENU(k,2), ...
            results.positionENU(k,3));
        addpoints(traveledTop,results.positionENU(k,1),results.positionENU(k,2));
        update_markers_and_title(k);
        drawnow limitrate;
        simulatedStepSec = results.timeSec(k)-results.timeSec(previousIndex);
        if simulatedStepSec > 0
            pause(simulatedStepSec/cfg.visualization.playbackSpeed);
        end
        previousIndex = k;
    end
end
layoutTitle = title(layout,sprintf( ...
    ['AOI center: E %.1f m, N %.1f m | %.1f m x %.1f m | ' ...
    'physical transit/search: %.2f/%.2f m/s | playback: %.1fx'], ...
    aoi.centerEN(1),aoi.centerEN(2),aoi.widthM,aoi.heightM, ...
    cfg.flight.transitSpeedMps,cfg.flight.searchSpeedMps, ...
    cfg.visualization.playbackSpeed));
set(layoutTitle,'Color','k');

    function update_markers_and_title(k)
        set(vehicleMarker3D,'XData',results.positionENU(k,1), ...
            'YData',results.positionENU(k,2),'ZData',results.positionENU(k,3));
        set(targetMarker3D,'XData',results.targetENU(k,1), ...
            'YData',results.targetENU(k,2),'ZData',results.targetENU(k,3));
        set(vehicleMarkerTop,'XData',results.positionENU(k,1), ...
            'YData',results.positionENU(k,2));
        set(targetMarkerTop,'XData',results.targetENU(k,1), ...
            'YData',results.targetENU(k,2));
        phaseIndex = min(max(round(results.phaseCode(k)),1),numel(cfg.phase.names));
        title(ax3D,sprintf(['3-D mission phase: %s | t = %.1f s | target WP %d' ...
            ' (vertical scale exaggerated)'], ...
            cfg.phase.names{phaseIndex}, results.timeSec(k), ...
            round(results.waypointIndex(k))),'Color','k');
    end
end

function style_axes(ax)
set(ax,'Color','w','XColor','k','YColor','k','ZColor','k', ...
    'GridColor',[0.75 0.75 0.75],'MinorGridColor',[0.85 0.85 0.85]);
end
