%{
Contributor(s): Wyatt Janosuek
Course number: ASEN 3801
File name: VelocityReferenceFeedback.m
Created: 03/31/2026
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- Goal: To calculate control forces and moments for our feedback control
system with a constraint of our velocity feedback
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- Methodology: The input is our state vector and time t. The output is our control
forces and control moments. These are based on our gains from our feedback
control system from 3.1 and 3.5
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%}

function [Fc, Gc] = VelocityReferenceFeedback(t, var)
%Quadrotor constants
m  = 0.068;
Ix = 5.8e-5;
Iy = 7.2e-5;
Iz = 1.0e-4;
g  = 9.81;

%State givens pulled
u = var(1);
v = var(2);
p = var(4);
q = var(5);
r = var(6);
phi = var(10);
theta = var(11);


%Inner Loop gains (3.1)
T1 = .5;
T2 = 0.05;
lambda1 = -1/T1;
lambda2 = -1/T2;

%Roll gains (3.1)
kp_roll = Ix * (lambda1 - lambda2);
kd_roll = Ix * (-(lambda1 + lambda2));

% Pitch gains (3.1)
kp_pitch = Iy * (lambda1 - lambda2);
kd_pitch = Iy * (-(lambda1 + lambda2));

%Outer Loop Gain (Problem 3.5)
k3 = .00005;

%Reference Velocity
V_ref = 0.5;
%Constant velocity and no accerlation
%Results in x = v*t = 1m
if t <= 2.0
    u_ref = V_ref;
    v_ref = V_ref;
%Deaccleration so it doesn't jump
elseif t <= 2.5
    u_ref = V_ref * (2.5 - t)/0.5;
    v_ref = V_ref * (2.5 - t)/0.5;%
%Rest after 2 seconds
else
    u_ref = 0;
    v_ref = 0;
end


xddot_des = -kpx*(x - x_ref) - kdx*(u - u_ref);
zddot_des = -kpz*(z - z_ref) - kdz*(w - w_ref);

theta_ref = -xddot_des / g;
Zc = (-m*g + m*zddot_des) / cos(theta_ref);

Fc = [0; 0; Zc];

%lateral velocity control
phi_ref   =  k3 * (v_ref - v);

%Roll moment
L = -kd_roll * p - kp_roll * (phi - phi_ref);
%Pitch moment
M = -kd_pitch * q - kp_pitch * (theta - theta_ref);
%Yaw moment
N = -1e-4 * r;

%Control Moments
Gc = [L; M; N];
end
