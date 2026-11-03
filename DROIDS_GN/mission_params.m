function cfg = mission_params()
%MISSION_PARAMS  All mission inputs in one place: run options, location, drone, flight
%   profile, camera, guidance, timing, controller, power, wind, noise and sim settings.
%   Flat cfg.* fields are read by the gn planner and the Simulink model (keep those names);
%   grouped structs (cfg.drone, cfg.wind, ...) are read by power/main_motorf_robust.

%% Run options (run_mission)
cfg.run.random_aoi   = true;     % true: random polygon AOI; false: aoiCorner/aoiSize rectangle
cfg.run.simulink     = false;    % true: also run gncc_uav_toolbox and overlay the flown path
cfg.run.power        = true;     % true: also run power/main_motorf_robust (motor forces + power draw)
cfg.run.save_figs    = true;     % true: save PNGs to gn/figures and power/figures
cfg.run.area_m       = 30;       % m, side of the square test area the random AOI sits in
cfg.run.min_size_m   = 5;        % m, smallest random AOI extent
cfg.run.start_dist_m = [10 20];  % m, [min max] home distance from the random AOI
cfg.run.z_exag       = 4;        % vertical exaggeration on the 3D plots

%% Location
cfg.home      = [0; 0; 0];           % m, NED home / takeoff point
cfg.lla0      = [40.0, -105.0, 1600];% deg, deg, m: WGS84 origin of NED (arbitrary, not a survey)
cfg.aoiCorner = [6; 2];              % m, [N; E] corner of the rectangular AOI
cfg.aoiSize   = [5; 5];              % m, [N; E] size of the rectangular AOI
% cfg.aoiPoly   = [N1 N2 ...; E1 E2 ...];   % m, any simple polygon AOI (overrides the rectangle)
% cfg.laneHeading = deg2rad(0);             % rad, fixed lane heading (default: chosen automatically)

%% Drone (EMAX ECO III 2807 1300KV, AVIA 7035 3-blade, 6S)
cfg.mass = 3.2;              % kg, AUW (must be a row of Weight and Power.xlsx Sheet3)
cfg.g    = 9.81;             % m/s^2
cfg.drone.Ix = 0.0045813;    % kg m^2 (still from the lighter run_all_designs vehicle)
cfg.drone.Iy = 0.0045833;
cfg.drone.Iz = 0.0080147;
cfg.drone.d  = 0.18785;      % m, arm length
cfg.drone.km = 0.0045;       % motor torque constant
cfg.drone.mu = 2e-6;         % rotational drag coefficient
cfg.motor.f_max = 22.4;      % N, full-throttle thrust per motor (2290 g)
cfg.motor.f_min = 0;         % N, motors can't push down
cfg.motor.kT    = 3.27e-6;   % N/(rad/s)^2, f = kT*Omega^2 fit to the thrust table
cfg.motor.tau_m = 0.04;      % s, motor + prop spin-up time constant (ASSUMED)

%% Aerodynamics
cfg.aero.rho  = 1.03;            % kg/m^3, air density at ~1650 m (Boulder)
cfg.aero.Cd   = 1.0;             % body drag coefficient (ASSUMED, bluff frame)
cfg.aero.area = 0.04;            % m^2, frontal area (ASSUMED, measure from CAD)
cfg.aero.r_cp = [0; 0; -0.02];   % m, center of pressure rel. to CG, body FRD (ASSUMED)

%% Flight profile
cfg.altitude  = 1.5;         % m AGL, search altitude
cfg.hMax      = 2;           % m AGL, ceiling
cfg.vNom      = 0.4;         % m/s, planned cruise speed
cfg.vMax      = 0.5;         % m/s
cfg.landSpeed = 0.15;        % m/s
cfg.aNom      = 0.3;         % m/s^2, smoothed trajectory accel (leaves aMax headroom for feedback)
cfg.aMax      = 0.6;         % m/s^2
cfg.tiltMax   = deg2rad(10); % rad

%% Camera / coverage
% Use the lower end of the selected cross-track FOV range.
% The camera's wide image dimension must point across the lanes.
cfg.cameraCrossTrackFOV = 75;     % deg
cfg.cameraLaneOverlap   = 0.20;   % Development assumption: 30% !!!EXPLAIN FOR DR2

%% Guidance (Simulink)
cfg.kPos             = 0.8;
cfg.kVel             = 2.5;
cfg.wpTol            = 0.08;    % m
cfg.homeTol          = 0.05;    % m
cfg.arrivalSpeed     = 0.04;    % m/s
cfg.lookahead        = 0.5;     % m
cfg.transitionRadius = 0.08;    % m

%% Timing
cfg.ctrlTs   = 0.02;    % s
cfg.navTs    = 0.02;    % s (later change to 0.2)
cfg.animTs   = 0.1;     % s
cfg.stopTime = 600;     % s

%% GPS
cfg.useGPS      = false;
cfg.gpsPosSigma = 0;    % m (start zero-noise for coordinate tests)
cfg.gpsVelSigma = 0;    % m/s

%% Controller, power sim: outer body-tilt cascade (run_all_designs.m)
cfg.ctrl.kpx_pos = 0.635;  cfg.ctrl.kpu = 1.153;  cfg.ctrl.kiu = 0.05969;
cfg.ctrl.kpy_pos = 0.635;  cfg.ctrl.kpv = 1.153;  cfg.ctrl.kiv = 0.05969;
cfg.ctrl.kpz_pos = 4.441;  cfg.ctrl.kpw = 9.992;  cfg.ctrl.kiw = 2.064;
cfg.ctrl.v_cap   = 1.0;    % m/s, velocity-command cap (lets the position loop correct drift)

%% Controller, power sim: inner attitude loop
cfg.att.kp_roll  = 0.1193937;  cfg.att.kd_roll  = 0.02112067;  cfg.att.ki_roll  = 0.02566814;
cfg.att.kp_pitch = 0.1193937;  cfg.att.kd_pitch = 0.02112067;  cfg.att.ki_pitch = 0.02566814;
cfg.att.Tpsi     = [0.5 0.2];  % s, yaw-hold pole time constants
cfg.att.tilt_lim = deg2rad(30);% rad, tilt command limit

%% Power (Weight and Power.xlsx, Sheet3 row at cfg.mass)
cfg.power.xlsx = fullfile(fileparts(mfilename('fullpath')), 'power', 'Weight and Power.xlsx');

%% Wind (cases with wind = true)
cfg.wind.speed  = 1;     % m/s, mean wind speed
cfg.wind.to_deg = 90;    % deg, heading the wind blows TOWARD (90 = east)
cfg.wind.W20    = 5;     % m/s, Dryden intensity (wind at 20 ft): ~7.7 light, ~15 moderate, ~23 severe
cfg.wind.seed   = 1;     % random seed (repeatable gusts)

%% Sensor / estimate noise (cases with noise = true), first-order colored: 1-sigma, correlation time
cfg.noise.pos  = 0.05;          cfg.noise.tau_pos  = 0.5;    % m,     s
cfg.noise.att  = deg2rad(0.3);  cfg.noise.tau_att  = 0.1;    % rad,   s
cfg.noise.vel  = 0.05;          cfg.noise.tau_vel  = 0.2;    % m/s,   s
cfg.noise.rate = deg2rad(0.5);  cfg.noise.tau_rate = 0.01;   % rad/s, s (gyro after ~16 Hz LPF)
cfg.noise.seed = 2;

%% Power sim cases (same mission, gusts and noise)
cfg.cases = struct( ...
  'name',  {'Ideal',  'Wind + gusts', 'Wind + gusts + noise'}, ...
  'wind',  {false,    true,           true}, ...
  'noise', {false,    false,          true});

%% Power sim
cfg.psim.dt        = 0.004;            % s, fixed-step RK4 (250 Hz)
cfg.psim.t_end     = Inf;              % s, Inf = whole mission
cfg.psim.plot_case = 2; % case shown in the detailed figures

%% Derived
cfg.cameraFootprintWidth = 2*cfg.altitude*tand(cfg.cameraCrossTrackFOV/2);   % m, flat ground
cfg.laneSpacing = cfg.cameraFootprintWidth*(1 - cfg.cameraLaneOverlap);      % m, overlapping passes

assert(cfg.altitude > 0 && cfg.altitude <= cfg.hMax);
assert(cfg.vNom > 0 && cfg.vNom <= cfg.vMax);
assert(cfg.landSpeed > 0 && cfg.landSpeed <= cfg.vMax);
assert(all(cfg.aoiSize > 0) && cfg.laneSpacing > 0);
assert(abs(cfg.navTs/cfg.ctrlTs - round(cfg.navTs/cfg.ctrlTs)) < 1e-9);
end
