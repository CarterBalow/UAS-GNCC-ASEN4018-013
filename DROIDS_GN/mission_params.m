function cfg = mission_params()
%MISSION_PARAMS  All mission inputs. Flat cfg.* fields feed the planner and Simulink
%   (keep the names); grouped structs (cfg.drone, cfg.wind, ...) feed power/.

%% Run options (run_mission)
cfg.run.random_aoi   = true;     % random polygon AOI (false: aoiCorner/aoiSize rectangle)
cfg.run.simulink     = false;    % also fly the Simulink model
cfg.run.power        = true;     % also run the power analysis
cfg.run.noise_figs   = true;     % also plot sensor noise vs. filtered estimate
cfg.run.save_figs    = true;     % save PNGs
cfg.run.area_m       = 30;       % m, test-area side for the random AOI
cfg.run.min_size_m   = 5;        % m, smallest random AOI extent
cfg.run.start_dist_m = [10 20];  % m, [min max] home distance from the random AOI
cfg.run.z_exag       = 4;        % 3D plot vertical exaggeration

%% Location / AOI (local NED)
cfg.home       = [0; 0; 0];             % m, takeoff point
cfg.lla0       = [40.0, -105.0, 1600];  % deg, deg, m: NED origin
cfg.aoiCorner  = [6; 2];                % m, [N; E] rectangle corner
cfg.aoiSize    = [5; 5];                % m, [N; E] rectangle size
% cfg.aoiPoly  = [N1 N2 ...; E1 E2 ...];  % m, any simple polygon (overrides rectangle)
cfg.aoiBuffer  = 0.5;                   % m, search the AOI grown by this on every side
cfg.laneExtend = true;                  % run lanes one turn radius past the boundary
% cfg.laneHeading = deg2rad(0);         % rad, fixed lane heading (default: auto)

%% Drone (EMAX ECO III 2807 1300KV, AVIA 7035, 6S)
cfg.mass = 3.2;              % kg, AUW (must be a row of Weight and Power.xlsx Sheet3)
cfg.g    = 9.81;             % m/s^2
cfg.drone.Ix = 0.0045813;    % kg m^2 (from the lighter vehicle, update)
cfg.drone.Iy = 0.0045833;    % kg m^2
cfg.drone.Iz = 0.0080147;    % kg m^2
cfg.drone.d  = 0.18785;      % m, arm length
cfg.drone.km = 0.0045;       % motor torque constant
cfg.drone.mu = 2e-6;         % rotational drag coefficient
cfg.motor.f_max = 22.4;      % N, max thrust per motor
cfg.motor.f_min = 0;         % N, min thrust per motor
cfg.motor.kT    = 3.27e-6;   % N/(rad/s)^2, thrust = kT*Omega^2
cfg.motor.tau_m = 0.04;      % s, motor spin-up time constant (assumed)

%% Aerodynamics (assumed)
cfg.aero.rho  = 1.03;            % kg/m^3, air density (Boulder)
cfg.aero.Cd   = 1.0;             % body drag coefficient
cfg.aero.area = 0.04;            % m^2, frontal area
cfg.aero.r_cp = [0; 0; -0.02];   % m, center of pressure rel. to CG (body FRD)

%% Flight profile
cfg.altitude  = 1.5;         % m AGL, search altitude
cfg.hMax      = 2;           % m AGL, ceiling
cfg.vNom      = 0.4;         % m/s, planned speed
cfg.vMax      = 0.5;         % m/s, speed limit
cfg.landSpeed = 0.15;        % m/s
cfg.aNom      = 0.3;         % m/s^2, planned accel
cfg.aMax      = 0.6;         % m/s^2, accel limit
cfg.tiltMax   = deg2rad(10); % rad, Simulink tilt limit

%% Camera (wide image axis across the lanes)
cfg.cameraCrossTrackFOV = 75;     % deg, low end of the selected range
cfg.cameraLaneOverlap   = 0.20;   % overlap between lanes !!!EXPLAIN FOR DR2

%% Guidance (Simulink)
cfg.kPos             = 0.8;     % position gain
cfg.kVel             = 2.5;     % velocity gain
cfg.wpTol            = 0.08;    % m, waypoint reached
cfg.homeTol          = 0.05;    % m, home reached
cfg.arrivalSpeed     = 0.04;    % m/s, counts as stopped
cfg.lookahead        = 0.5;     % m
cfg.transitionRadius = 0.08;    % m

%% Timing
cfg.ctrlTs   = 0.02;    % s, controller step
cfg.navTs    = 0.02;    % s, nav step (later 0.2)
cfg.animTs   = 0.1;     % s, animation step
cfg.stopTime = 600;     % s, Simulink stop time

%% GPS (Simulink)
cfg.useGPS      = false;
cfg.gpsPosSigma = 0;    % m
cfg.gpsVelSigma = 0;    % m/s

%% Outer loop (power sim): position -> velocity PI, per axis
% 10-29 pole placement; horizontal kpos, kp x1.5 and ki x4 (interim, D&C redesigning).
% Don't use the 10-27 tuned z gains: they ring at 3.7 Hz with the motor lag.
cfg.ctrl.kpx_pos = 0.9525; cfg.ctrl.kpu = 1.7295; cfg.ctrl.kiu = 0.23876;
cfg.ctrl.kpy_pos = 0.9525; cfg.ctrl.kpv = 1.7295; cfg.ctrl.kiv = 0.23876;
cfg.ctrl.kpz_pos = 4.441;  cfg.ctrl.kpw = 9.992;  cfg.ctrl.kiw = 2.064;
cfg.ctrl.v_cap   = 1.0;    % m/s, velocity command cap
cfg.ctrl.acc_ff  = true;   % feed mission acceleration forward

%% Inner loop (power sim): roll/pitch PID by pole placement with the motor lag
% tau*I*s^4 + I*s^3 + kd*s^2 + kp*s + ki = 0, so the poles must sum to -1/tau_m.
cfg.att.poles = [-7+7i, -7-7i, -7, -4];   % rad/s
assert(abs(real(sum(cfg.att.poles)) + 1/cfg.motor.tau_m) < 1e-6, ...
       'cfg.att.poles must sum to -1/tau_m = %.1f', -1/cfg.motor.tau_m);
c_att = real(poly(cfg.att.poles));
g_r = cfg.motor.tau_m*cfg.drone.Ix*c_att(3:5);
g_p = cfg.motor.tau_m*cfg.drone.Iy*c_att(3:5);
cfg.att.kd_roll  = g_r(1);  cfg.att.kp_roll  = g_r(2);  cfg.att.ki_roll  = g_r(3);
cfg.att.kd_pitch = g_p(1);  cfg.att.kp_pitch = g_p(2);  cfg.att.ki_pitch = g_p(3);
cfg.att.Tpsi     = [0.5 0.2];   % s, yaw-hold time constants
cfg.att.tilt_lim = deg2rad(30); % rad, tilt command limit

%% Power
cfg.power.xlsx = fullfile(fileparts(mfilename('fullpath')), 'power', 'Weight and Power.xlsx');

%% Wind (cases with wind = true)
cfg.wind.speed  = 1;     % m/s, mean wind
cfg.wind.to_deg = 90;    % deg, direction blown toward (90 = east)
cfg.wind.W20    = 5;     % m/s, Dryden gust intensity (7.7 light, 15 moderate)
cfg.wind.seed   = 1;

%% Simple noise model (cases with noise = true): 1-sigma, correlation time
cfg.noise.pos  = 0.05;          cfg.noise.tau_pos  = 0.5;    % m, s
cfg.noise.att  = deg2rad(0.3);  cfg.noise.tau_att  = 0.1;    % rad, s
cfg.noise.vel  = 0.05;          cfg.noise.tau_vel  = 0.2;    % m/s, s
cfg.noise.rate = deg2rad(0.5);  cfg.noise.tau_rate = 0.01;   % rad/s, s
cfg.noise.seed = 2;

%% ESKF in the loop (cases with eskf = 'pvt' or 'rtk'); sensor specs in power/eskf16/eskf16_specs.m
cfg.eskf.board       = 'pixhawk6c';
cfg.eskf.t_prearm    = 2.0;                          % s, still time before takeoff
cfg.eskf.gps_dt      = 0.2;                          % s, GPS rate (5 Hz)
cfg.eskf.baro_dt     = 0.02;                         % s, baro rate (50 Hz)
cfg.eskf.use_baro    = true;                         % false = GPS-only height
cfg.eskf.gyro_lpf_hz = 30;                           % Hz, gyro low-pass to the controller
cfg.eskf.yaw0_err    = deg2rad(5);                   % rad, initial heading error
cfg.eskf.bg0         = deg2rad([0.15; -0.10; 0.20]); % rad/s, true gyro bias
cfg.eskf.ba0         = [0.05; -0.03; 0.08];          % m/s^2, true accel bias
cfg.eskf.seed        = 3;

%% Power sim cases (eskf: '' = controller sees truth)
cfg.cases = struct( ...
  'name',  {'Ideal',  'Wind + gusts', 'Wind + gusts + ESKF (GPS)', 'Wind + gusts + ESKF (RTK)'}, ...
  'wind',  {false,    true,           true,                        true}, ...
  'noise', {false,    false,          false,                       false}, ...
  'eskf',  {'',       '',             'pvt',                       'rtk'});

%% Power sim
cfg.psim.dt        = 0.004;   % s, RK4 step (250 Hz)
cfg.psim.t_end     = Inf;     % s, Inf = whole mission
cfg.psim.plot_case = 2;       % case shown in the detailed figures

%% Derived
cfg.cameraFootprintWidth = 2*cfg.altitude*tand(cfg.cameraCrossTrackFOV/2);   % m
cfg.laneSpacing = cfg.cameraFootprintWidth*(1 - cfg.cameraLaneOverlap);      % m
cfg.laneExt = cfg.laneExtend*cfg.vNom^2/cfg.aNom;                            % m, turn radius

assert(cfg.altitude > 0 && cfg.altitude <= cfg.hMax);
assert(cfg.vNom > 0 && cfg.vNom <= cfg.vMax);
assert(cfg.landSpeed > 0 && cfg.landSpeed <= cfg.vMax);
assert(all(cfg.aoiSize > 0) && cfg.laneSpacing > 0);
assert(abs(cfg.navTs/cfg.ctrlTs - round(cfg.navTs/cfg.ctrlTs)) < 1e-9);
end
