# Multirotor Guidance and Navigation Simulation

This project models the UAS portion of the ASEN 4018 mission: take off from home, transit to a randomized area of interest (AOI), fly an axis-aligned serpentine search, return home, and land. All position vectors use local **ENU [East, North, Up] in meters**.

## Run

From this folder in MATLAB:

```matlab
results = run_multirotor_mission();       % new random AOI
results = run_multirotor_mission(42);     % repeatable AOI/noise seed
```

The run prints automated checks and opens a 3-D ENU animation with the AOI ground footprint, search-altitude plane, planned and traveled trajectories, current vehicle and target, a supporting top-down view, and altitude/3-D-speed histories. Plot limits are calculated from the complete plan and simulation so home, takeoff, AOI search, return, and landing remain visible for every randomized AOI. The 3-D plot deliberately exaggerates the vertical scale so a 0-2 m altitude range remains visible beside the longer horizontal transit. The main model is `multirotor_gn_model.slx`. If it is absent, the entry point generates it by calling `build_multirotor_simulink_model`.

Run `report = self_test_multirotor_mission();` for a repeatable, non-graphical end-to-end regression check plus an ordinary-run AOI randomization check.

## Change settings without editing multiple files

Edit defaults in `mission_config.m`, or pass only the fields to override:

```matlab
o.randomSeed = 12;
o.aoi.centerDistanceRangeM = [8 12];
o.aoi.sizeRangeM = [5 5];
o.aoi.trackSpacingM = 0.75;
o.flight.transitAltitudeM = 1.8;
o.flight.searchAltitudeM = 1.2;
o.flight.transitSpeedMps = 0.25;  % physical vehicle speed
o.flight.searchSpeedMps = 0.15;   % physical vehicle speed
o.visualization.playbackSpeed = 4; % 4 simulated seconds per real second
results = run_multirotor_mission(o);
```

`mission_config` rejects altitude or speed settings above the RFP limits of 2 m AGL and 0.5 m/s.

The physical speed settings affect Simulink dynamics and mission duration. `visualization.playbackSpeed` affects only how quickly completed simulation data are animated; `1` is approximately real time and larger values play faster. `visualization.frameStride` controls drawing density without changing the physical simulation.

## Plot line styles

- Gray dashed lines are the planned waypoint route in 3-D and top-down views.
- Solid red lines are the simulated trajectory.
- Red dashed horizontal lines are the RFP maximum altitude and maximum speed.
- Cyan dotted outlines show the search-altitude plane and its connection to the AOI ground footprint.

## Model scope

The Simulink plant is a kinematic point mass with a first-order velocity response on each ENU axis. The Up-position integrator includes a zero-altitude ground constraint. Landing completion uses true position and a separate tight ground-contact threshold so navigation noise cannot end the run in midair. The model captures finite speed/turn response but not attitude, motors, aerodynamics, wind, terrain, collision avoidance, vision, a rover, or manual-override hardware. Navigation adds configurable position noise and low-pass filtering. MATLAB handles setup, randomized AOI and route generation, plotting/animation, and verification.

The original `testGNModel.slx` and `init_model.m` are retained as the starter reference. The new model preserves their Guidance -> Point Mass -> Navigation loop, first-order lag concept, noisy navigation feedback, and acceptance-radius waypoint switching, while adding mission phases and requirement-aware behavior.
