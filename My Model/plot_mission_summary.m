function fig = plot_mission_summary(results, cfg, mission, aoi)
%PLOT_MISSION_SUMMARY Fast, non-animated mission visualization.

cfg.visualization.animate = false;
fig = animate_mission(results,cfg,mission,aoi);
end
