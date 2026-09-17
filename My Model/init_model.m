% ---- SIMULATION SETUP ----
Ts = 0.01; % Sample time (100 Hz simulation)

% ---- MISSION GUIDANCE ----
% A simple 3D box mission path [X, Y, Z] (NED coordinates: Z is negative for altitude)
waypoints = [  0,   0,   0;
              10,   0, -5;
              10,  10, -5;
               0,  10, -5;
               0,   0, -5 ];

cruise_speed = 2.0;       % Speed limit (m/s)
acceptance_radius = 0.5;  % Distance to target to switch waypoints (m)

% ---- VEHICLE PLANT HACK ----
tau_delay = 0.2; % Time constant approximating drone physical lag (seconds)

% ---- NAV SENSOR NOISE ----
gps_noise_variance = 0.05; 