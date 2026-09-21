clc; clear; close all;
this_dir = fileparts(mfilename('fullpath'));
addpath(genpath(this_dir));
%% Vehicle + environment parameters (same as the body-tilt sim, for a
%% fair trade-study comparison)
params.g  = 9.81;
params.m  = .068;
params.Ix = 5.8e-5;
params.Iy = 7.2e-5;
params.Iz = 1.0e-4;
params.I  = diag([params.Ix; params.Iy; params.Iz]);
params.d  = .06;      % arm length
params.km = .0024;    % motor torque constant
params.nu = 1e-3;     % translational drag coefficient
params.mu = 2e-6;     % rotational drag coefficient
%% Reference command: hold z=0, y=0, cruise at u_max to x = 1000 m
ref.x = 1000;
ref.y = 0;  ref.v = 0;
ref.z = 0;  ref.w = 0;
ref.u_max   = 2;     % cruise speed cap (m/s)
ref.kpx_pos = 0.5;   % position error -> velocity command gain (1/s)
ref.kpu = 2;  ref.kiu = 1;   % velocity-loop PI gains
ref.kpy = 1.5; ref.kdy = 1.8;
ref.kpz = 2.0; ref.kdz = 2.5;
%% Initial condition: hover at origin, zero rates/attitude
% 24-element augmented state:
%   1:12  - real states
%   13:18 - unused Fc/Gc slots (overwritten below for plotting)
%   19:21 - [xi_phi; xi_theta(unused); xi_u]
%   22:24 - [gamma; gamma_dot; gammai]  (propeller-tilt actuator state)
var0 = zeros(24,1);
tspan = [0, 10];
%% Closed-loop nonlinear simulation (thrust-vectoring design)
[t, state] = ode45(@(t, var) closedloopEOMthrustvec(t, var, params, ref), ...
                    tspan, var0);
%% Post-process: recompute Fc/Gc/motor-force history for plotting (not
%% part of the integrated state)
Fc_hist = zeros(length(t), 3);
Gc_hist = zeros(length(t), 3);
motor_forces_hist = zeros(length(t), 4);
for i = 1:length(t)
    [Fc_i, Gc_i] = closedloopthrustvec(state(i,:), params, ref);
    Fc_hist(i,:) = Fc_i';
    Gc_hist(i,:) = Gc_i';
    motor_forces_hist(i,:) = ComputeMotorForces(Fc_i, Gc_i, params.d, params.km)';
end
state(:,13:15) = Fc_hist;
state(:,16:18) = Gc_hist;
%% Plot everything
PlotAllStates(t, state, 'Thrust-Vectoring Quadrotor - Forward Flight');
PlotMotorForces(t, motor_forces_hist, 'Motor Thrust Forces - Thrust Vectoring');
PlotPropellerTilt(t, state(:,22), state(:,23), 'Propeller Tilt - Thrust Vectoring');
fprintf('Final position: x=%.3f  y=%.3f  z=%.3f (target x=%.1f)\n', ...
        state(end,1), state(end,2), state(end,3), ref.x);
fprintf('Final body pitch: %.2f deg   Final gamma: %.2f deg\n', ...
        rad2deg(state(end,5)), rad2deg(state(end,22)));
