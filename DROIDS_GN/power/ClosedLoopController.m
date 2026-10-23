function [Fc, Gc, int_dot, dbg] = ClosedLoopController(var, params, ref)
% ClosedLoopController - Body-tilt ("thrust input") quadrotor controller.
%
% Outer loop (all three axes, built on INERTIAL position/velocity):
%   x, y : position -> saturated velocity command -> PI velocity loop
%            vx_cmd = sat(kpx_pos*(ref.x - xE), u_max)
%            xddot  = -kpu*(xE_dot - vx_cmd) - kiu*xi_u
%            vy_cmd = sat(kpy_pos*(ref.y - yE), v_max)
%            yddot  = -kpv*(yE_dot - vy_cmd) - kiv*xi_v
%   z    : same cascade, NED (z down, so climbing is negative z)
%            vz_cmd = sat(kpz_pos*(ref.z - zE), w_max)
%            zddot  = -kpw*(zE_dot - vz_cmd) - kiw*xi_z
%
% Target modes (per axis): ref.x_mode / ref.y_mode / ref.z_mode set to
%   'position' (default) - fly to ref.x / ref.y / ref.z (cascade above)
%   'velocity'           - skip the position loop and hold a constant
%                          inertial velocity ref.vx / ref.vy / ref.vz (m/s),
%                          still saturated at u_max / v_max / w_max:
%                            v_cmd = sat(ref.vy, v_max)
%   The PI velocity loop underneath is identical in both modes.
%
% Heading-frame rotation: the inertial x/y accel commands are rotated
% through psi so the tilt commands stay correct when yaw is not zero:
%   [ax_H; ay_H] = [cos psi, sin psi; -sin psi, cos psi] * [xddot; yddot]
%   theta_ref = -ax_H/g ,  phi_ref = ay_H/g      (small-angle inversion)
%
% Thrust: projects the vertical requirement through the ACTUAL tilt,
%   Zc = -m*(g - zddot) / (cos(phi)*cos(theta))
% (previously only /cos(theta_ref), so any roll -- i.e. any y maneuver --
% would have cost altitude).
%
% Inner loop: roll/pitch PID (pole placement, unchanged), plus a yaw
% angle hold (PD) replacing the old yaw rate-damping-only term, so a yaw
% offset is driven back to ref.psi instead of persisting.
%
% Anti-windup: each velocity integral (xi_u, xi_v) freezes while its own
% tilt command is saturated; xi_z freezes while the thrust command is
% outside [T_min, T_max] of the motors (only matters if params.f_max set).
%
% State layout (23 elements):
%   1:12  real states [xE yE zE phi theta psi uB vB wB p q r]
%   13:18 unused Fc/Gc slots (plotting only)
%   19:23 integrals [xi_phi; xi_theta; xi_u; xi_v; xi_z]
%         (xi_u, xi_v, xi_z are integrals of the x/y/z VELOCITY errors)
%
% Outputs:
%   Fc      - [0;0;Zc] (N),  Gc - [L;M;N] (N*m)
%   int_dot - d/dt of states 19:23
%   dbg     - struct of intermediate commands (for plotting/debug)

m = params.m; g = params.g;
Ix = params.Ix; Iy = params.Iy; Iz = params.Iz;

xE = var(1); yE = var(2); zE = var(3);
phi = var(4); theta = var(5); psi = var(6);
uB = var(7); vB = var(8); wB = var(9);
p = var(10); q = var(11); r = var(12);
xi_phi   = var(19);
xi_theta = var(20);
xi_u     = var(21);
xi_v     = var(22);
xi_z     = var(23);

% Defaults so older ref structs still run
if ~isfield(ref, 'psi'),   ref.psi = 0; end
if ~isfield(ref, 'v_max'), ref.v_max = ref.u_max; end
if ~isfield(ref, 'w_max'), ref.w_max = 1; end
if ~isfield(params, 'f_max'), params.f_max = Inf; end   % per-motor thrust limit (N)
if ~isfield(ref, 'x_mode'), ref.x_mode = 'position'; end
if ~isfield(ref, 'y_mode'), ref.y_mode = 'position'; end
if ~isfield(ref, 'z_mode'), ref.z_mode = 'position'; end

% --- Inertial velocity: V^E = Q_{E/B} * V^B (3-2-1 Euler) ---
cph = cos(phi); sph = sin(phi);
cth = cos(theta); sth = sin(theta);
cps = cos(psi); sps = sin(psi);
Q_EB = [cth*cps, sph*sth*cps - cph*sps, cph*sth*cps + sph*sps;
        cth*sps, sph*sth*sps + cph*cps, cph*sth*sps - sph*cps;
        -sth,    sph*cth,               cph*cth];
VE = Q_EB * [uB; vB; wB];
xE_dot = VE(1); yE_dot = VE(2); zE_dot = VE(3);

tilt_limit = deg2rad(30);

% --- x: (position -> ) velocity command -> PI ---
vx_cmd = VelocityCommand(ref.x_mode, ref, 'x', 'vx', ref.kpx_pos, xE, ref.u_max);
ex_v = xE_dot - vx_cmd;
xddot_des = -ref.kpu*ex_v - ref.kiu*xi_u;

% --- y: (position -> ) velocity command -> PI (mirrors x) ---
vy_cmd = VelocityCommand(ref.y_mode, ref, 'y', 'vy', ref.kpy_pos, yE, ref.v_max);
ey_v = yE_dot - vy_cmd;
yddot_des = -ref.kpv*ey_v - ref.kiv*xi_v;

% --- z: (position -> ) climb-rate command -> PI (mirrors x, y) ---
vz_cmd = VelocityCommand(ref.z_mode, ref, 'z', 'vz', ref.kpz_pos, zE, ref.w_max);
ez_v = zE_dot - vz_cmd;
zddot_des = -ref.kpw*ez_v - ref.kiw*xi_z;

% --- Rotate horizontal accel commands into the heading frame ---
ax_H =  cps*xddot_des + sps*yddot_des;
ay_H = -sps*xddot_des + cps*yddot_des;

theta_ref_raw = -ax_H / g;
phi_ref_raw   =  ay_H / g;
theta_ref = max(min(theta_ref_raw, tilt_limit), -tilt_limit);
phi_ref   = max(min(phi_ref_raw,   tilt_limit), -tilt_limit);

% --- Thrust: vertical requirement projected through actual tilt ---
tilt_cos = max(cph*cth, cos(tilt_limit + deg2rad(15)));   % guard vs /0
T_cmd = m*(g - zddot_des) / tilt_cos;                     % total thrust (+)
T_min = 0;  T_max = 4*params.f_max;
T = max(min(T_cmd, T_max), T_min);
Zc = -T;
Fc = [0; 0; Zc];

% --- Anti-windup ---
x_sat = abs(theta_ref_raw) >= tilt_limit;
y_sat = abs(phi_ref_raw)   >= tilt_limit;
z_sat = (T_cmd > T_max && ez_v > 0) || (T_cmd < T_min && ez_v < 0);
int_u = ex_v * ~x_sat;
int_v = ey_v * ~y_sat;
int_z = ez_v * ~z_sat;

% --- Inner loop: roll/pitch PID (pole placement, unchanged) ---
T1 = .5; T2 = .05; T3 = 1;
l1 = -1/T1; l2 = -1/T2; l3 = -1/T3;

% kp_roll  = Ix*(l1*l2 + l1*l3 + l2*l3);
% kd_roll  = -Ix*(l1 + l2 + l3);
% ki_roll  = -Ix*l1*l2*l3;
% 
% kp_pitch = Iy*(l1*l2 + l1*l3 + l2*l3);
% kd_pitch = -Iy*(l1 + l2 + l3);
% ki_pitch = -Iy*l1*l2*l3;

kp_roll  = .1193937;
kd_roll  = .02112067;
ki_roll  = .02566814;

kp_pitch = .1193937;
kd_pitch = .02112067;
ki_pitch = .02566814;

% --- Yaw angle hold: Iz*s^2 + kd*s + kp, poles at -1/Tpsi1, -1/Tpsi2 ---
Tpsi1 = 0.5; Tpsi2 = 0.2;
kp_yaw = Iz*(1/Tpsi1)*(1/Tpsi2);
kd_yaw = Iz*(1/Tpsi1 + 1/Tpsi2);

phi_err   = phi   - phi_ref;
theta_err = theta - theta_ref;
psi_err   = atan2(sin(psi - ref.psi), cos(psi - ref.psi));   % wrap to +/-pi

L = -kp_roll*phi_err    - kd_roll*p  - ki_roll*xi_phi;
M = -kp_pitch*theta_err - kd_pitch*q - ki_pitch*xi_theta;
N = -kp_yaw*psi_err     - kd_yaw*r;

Gc = [L; M; N];
int_dot = [phi_err; theta_err; int_u; int_v; int_z];

dbg = struct('vx_cmd', vx_cmd, 'vy_cmd', vy_cmd, 'vz_cmd', vz_cmd, 'xE_dot', xE_dot, ...
             'yE_dot', yE_dot, 'zE_dot', zE_dot, 'phi_ref', phi_ref, ...
             'theta_ref', theta_ref, 'T', T);

end
