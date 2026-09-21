function [Fc, Gc, int_dot, gamma_ddot, gamma_err] = closedloopthrustvec(var, params, ref)
% closedloopthrustvec - Thrust-vectoring version of the outer/inner loop
% controller: forward thrust comes from tilting the PROPELLERS by an
% angle gamma (a real actuator with its own 2nd-order dynamics), not
% from tilting the airframe. Body pitch theta is left to plain rate
% damping (no angle reference) since it's no longer doing the work of
% redirecting thrust.
%
% x-axis outer loop (cruise flight) -- same cascade as the body-tilt
% design, just inverted onto gamma instead of theta:
%   u_cmd     = kpx_pos*(ref.x - xE), saturated to +/- ref.u_max
%   xddot_des = -kpu*(uB - u_cmd) - kiu*xi_u
%   gamma_ref = xddot_des / g   (small-angle tilt-to-accel inversion)
%
% Propeller-tilt actuator: gamma is now a real dynamic state (var(22),
% with rate var(23)), so something has to drive it toward gamma_ref.
% ASSUMPTION (not given a physical spec): modeled as a normalized
% (unit-"inertia") 2nd-order servo via pole placement, same style as the
% roll/pitch inner loop. Replace Tg1/Tg2/Tg3 below with your actual
% tilt-mechanism bandwidth/damping once you have one -- these are just
% reasonable placeholder poles (faster than the outer velocity loop,
% slower than the fastest attitude pole) so the sim runs meaningfully.
%
% y, z outer loops: unchanged position/velocity PD.
% Roll (phi): unchanged PID body tilt (this design only replaces the
% pitch/forward axis with propeller vectoring).
%
% Inputs:
%   var    - 24-element state: [12 real states, 6 unused Fc/Gc slots,
%            xi_phi, xi_theta(unused), xi_u, gamma, gamma_dot, gammai]
%   params - struct: m, g, Ix, Iy, Iz
%   ref    - struct: x,y,z,v,w (setpoints), u_max, kpx_pos, kpu, kiu (x
%            cruise loop), kpy,kdy,kpz,kdz (y/z PD)
%
% Outputs:
%   Fc         - [Xc;0;Zc] control force (N) -- unused here, motor mixing
%                still only needs Fc(3); Xc is a consequence of gamma,
%                not an independent mixing input
%   Gc         - [L;M;N] control moment (N*m)
%   int_dot    - [d/dt(xi_phi); 0; d/dt(xi_u)]  (xi_theta unused, kept as
%                a zero to preserve the state-vector slot/layout)
%   gamma_ddot - propeller-tilt actuator command (rad/s^2) -> var_dot(23)
%   gamma_err  - gamma - gamma_ref -> integrates into var_dot(24) (gammai)

m = params.m; g = params.g;
Ix = params.Ix; Iy = params.Iy;

xE = var(1); yE = var(2); zE = var(3);
phi = var(4);
uB = var(7); vB = var(8); wB = var(9);
p = var(10); q = var(11); r = var(12);
xi_phi   = var(19);   % integral of roll-angle error
xi_u     = var(21);   % integral of forward-velocity error
gamma      = var(22); % propeller tilt angle
gamma_dot  = var(23); % propeller tilt rate
gammai     = var(24); % integral of gamma error

% --- x: position -> velocity command (saturated at cruise speed) ---
u_cmd = ref.kpx_pos * (ref.x - xE);
u_cmd = max(min(u_cmd, ref.u_max), -ref.u_max);

% --- x: PI velocity loop tracks u_cmd (zeros steady-state speed error) ---
u_err = uB - u_cmd;
xddot_des = -ref.kpu*u_err - ref.kiu*xi_u;

% Anti-windup: freeze xi_u while the required gamma exceeds the tilt
% limit, same reasoning as the body-tilt design's theta anti-windup.
gamma_ref_raw = xddot_des / g;
x_saturated = abs(gamma_ref_raw) >= deg2rad(30);
if x_saturated
    u_err_int = 0;
else
    u_err_int = u_err;
end

% --- y, z: position/velocity PD (unchanged) ---
yddot_des = -ref.kpy*(yE - ref.y) - ref.kdy*(vB - ref.v);
zddot_des = -ref.kpz*(zE - ref.z) - ref.kdz*(wB - ref.w);

% --- Invert for propeller tilt + thrust (gamma replaces theta_ref here) ---
gamma_ref = xddot_des / g;
phi_ref   = yddot_des / g;

tilt_limit = deg2rad(30);
gamma_ref = max(min(gamma_ref, tilt_limit), -tilt_limit);
phi_ref   = max(min(phi_ref,   tilt_limit), -tilt_limit);

% (fixed: was "cos(theta_ref)" -- theta_ref doesn't exist in this design,
% gamma_ref is what's actually redirecting thrust now)
Zc = (-m*g + m*zddot_des) / cos(gamma_ref);
Xc = m*xddot_des;  % for reference/logging only -- not fed back into mixing
Fc = [Xc; 0; Zc];

% --- Roll: unchanged PID body tilt, pole placement ---
T1 = .5; T2 = .05; T3 = 1;
lambda1 = -1/T1; lambda2 = -1/T2; lambda3 = -1/T3;

kp_roll  = Ix*(lambda1*lambda2 + lambda1*lambda3 + lambda2*lambda3);
kd_roll  = -Ix*(lambda1 + lambda2 + lambda3);
ki_roll  = -Ix*lambda1*lambda2*lambda3;

% pitch rate damping only (same T1/T2/T3 poles, but no angle reference --
% gamma is doing the work of forward thrust now, theta is just damped)
kd_pitch = -Iy*(lambda1 + lambda2 + lambda3);

k_yaw    = -.068*9.81*1e-4;  % unchanged (yaw damping only)

phi_err = phi - phi_ref;

L = -kp_roll*phi_err - kd_roll*p - ki_roll*xi_phi;
M = -kd_pitch*q;
N = k_yaw*r;

Gc = [L; M; N];
int_dot = [phi_err; 0; u_err_int];

% --- Propeller-tilt actuator: PID drives gamma -> gamma_ref ---
% ASSUMED poles (no physical servo spec given) -- normalized "inertia"
% of 1, i.e. gamma_ddot comes out directly in rad/s^2.
Tg1 = .1; Tg2 = .05; Tg3 = .2;
g1 = -1/Tg1; g2 = -1/Tg2; g3 = -1/Tg3;
kp_gamma = (g1*g2 + g1*g3 + g2*g3);
kd_gamma = -(g1 + g2 + g3);
ki_gamma = -g1*g2*g3;

gamma_err = gamma - gamma_ref;
gamma_ddot = -kp_gamma*gamma_err - kd_gamma*gamma_dot - ki_gamma*gammai;

end
