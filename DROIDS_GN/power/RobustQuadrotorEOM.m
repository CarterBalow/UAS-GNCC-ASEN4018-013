function x12_dot = RobustQuadrotorEOM(x12, P, motor_forces, wind_NED)
% RobustQuadrotorEOM - 12-state nonlinear quadrotor EOM (same equations and
% conventions as dynamics/QuadrotorEOM.m) with two additions:
%   1) Aerodynamic drag uses the AIR-RELATIVE velocity, so steady wind and
%      gusts push on the vehicle:
%        V_rel^B = V^B - Q_{B/E} W^E
%        F_aero^B = -nu * V_rel^B .* |V_rel^B|
%   2) The drag force acts at a center of pressure offset r_cp from the CG
%      (body frame), adding an aerodynamic moment r_cp x F_aero on top of
%      the rotational damping -mu * omega .* |omega|.
%
% Inputs:
%   x12          - [xE yE zE phi theta psi uB vB wB p q r]  (m, rad, m/s, rad/s)
%   P            - params struct: g, m, Ix, Iy, Iz, d, km, nu, mu, r_cp
%   motor_forces - ACTUAL motor thrusts [f1;f2;f3;f4] (N), after lag/saturation
%   wind_NED     - wind velocity in the inertial NED frame (m/s), 3x1
%
% Output:
%   x12_dot      - time derivative of x12

phi = x12(4); theta = x12(5); psi = x12(6);
VB    = x12(7:9);
omega = x12(10:12);
p = omega(1); q = omega(2); r = omega(3);

Ix = P.Ix; Iy = P.Iy; Iz = P.Iz;
f1 = motor_forces(1); f2 = motor_forces(2);
f3 = motor_forces(3); f4 = motor_forces(4);

% --- Control forces and moments (X-configuration, same as QuadrotorEOM) ---
s  = P.d/sqrt(2);
Zc = -(f1 + f2 + f3 + f4);
Lc = s*(-f1 - f2 + f3 + f4);
Mc = s*( f1 - f2 - f3 + f4);
Nc = P.km*(f1 - f2 + f3 - f4);

% --- Rotation Q_{E/B} (3-2-1 Euler) ---
cph = cos(phi);   sph = sin(phi);
cth = cos(theta); sth = sin(theta);
cps = cos(psi);   sps = sin(psi);
Q_EB = [cth*cps, sph*sth*cps - cph*sps, cph*sth*cps + sph*sps;
        cth*sps, sph*sth*sps + cph*cps, cph*sth*sps - sph*cps;
        -sth,    sph*cth,               cph*cth];

% --- Aerodynamics: drag on air-relative velocity, moment about the CG ---
V_rel   = VB - Q_EB.'*wind_NED(:);
F_aero  = -P.nu * V_rel .* abs(V_rel);
G_aero  = -P.mu * omega .* abs(omega) + cross(P.r_cp(:), F_aero);

% --- Total body-frame forces and moments ---
F = [0; 0; Zc] + F_aero;
G = [Lc; Mc; Nc] + G_aero;

% --- Kinematics ---
pos_dot = Q_EB*VB;
att_dot = [p + sph*tan(theta)*q + cph*tan(theta)*r;
           cph*q - sph*r;
           (sph*q + cph*r)/cth];

% --- Translational dynamics (body frame) ---
V_dot = -cross(omega, VB) + P.g*[-sth; cth*sph; cth*cph] + F/P.m;

% --- Rotational dynamics (Euler's equations, principal axes) ---
omega_dot = [(G(1) + (Iy - Iz)*q*r)/Ix;
             (G(2) + (Iz - Ix)*p*r)/Iy;
             (G(3) + (Ix - Iy)*p*q)/Iz];

x12_dot = [pos_dot; att_dot; V_dot; omega_dot];
end
