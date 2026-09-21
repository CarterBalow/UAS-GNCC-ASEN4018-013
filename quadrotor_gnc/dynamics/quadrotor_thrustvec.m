function var_dot = quadrotor_thrustvec(t, var, g, m, I, d, km, nu, mu, motor_forces, gamma)
% quadrotor_thrustvec - Nonlinear quadrotor EOM for the THRUST-VECTORING
% design: the propellers themselves tilt by an angle gamma (relative to
% the body frame) to redirect thrust, instead of tilting the whole
% airframe. Body pitch (theta) is free to stay near hover and is only
% rate-damped by the inner loop -- gamma is what produces forward thrust.
%
% NOTE: gamma is passed in explicitly (not read from `var`) because this
% function only ever receives the 12-element real-state slice (var(1:12)
% from the caller's 24-element augmented state) -- gamma lives in the
% caller's slots 22:24, outside that slice, so it has to come in as its
% own argument.
%
% This is a copy of QuadrotorEOM.m with the mixing replaced:
%   Xc = (f1+f2+f3+f4)*sin(gamma)   -- horizontal thrust component
%   Zc = -(f1+f2+f3+f4)*cos(gamma)  -- vertical thrust component
% (both driven by the SAME total thrust, just resolved through gamma --
% gamma doesn't add a new independent force, it redirects the existing
% one, same as body tilt did, just at the rotor level instead of the
% airframe level.)
%
% NOTE: this file's function name matches its filename (quadrotor_thrustvec),
% not "QuadrotorEOM" -- MATLAB/Octave dispatch a call by FILENAME, so a
% file with an internal function statement that doesn't match its own
% filename gets called by the filename regardless of what's written after
% "function ... = ". Naming this function "QuadrotorEOM" (as in the
% original draft) would NOT override dynamics/QuadrotorEOM.m -- any call
% to QuadrotorEOM(...) would keep silently resolving to that other file.

xE    = var(1);
yE    = var(2);
zE    = var(3);
phi   = var(4);   % Roll
theta = var(5);   % Pitch
psi   = var(6);   % Yaw
uB    = var(7);   % Body-frame velocity x
vB    = var(8);   % Body-frame velocity y
wB    = var(9);   % Body-frame velocity z
p     = var(10);  % Roll rate
q     = var(11);  % Pitch rate
r     = var(12);  % Yaw rate
% gamma (propeller tilt angle, rad) comes in as an argument -- see note above

% --- Extract inertia values ---
Ix = I(1,1);
Iy = I(2,2);
Iz = I(3,3);
% Ixz = 0 for symmetric quadrotor

% --- Motor forces ---
f1 = motor_forces(1);
f2 = motor_forces(2);
f3 = motor_forces(3);
f4 = motor_forces(4);

% --- Control forces and moments ---
% (fixed: was "(1 + f2 + f3 + f4)" -- missing f1 -- and cosd(gamma),
% which mixed degrees into a radian-valued state, same class of bug as
% the one already fixed in QuadrotorEOM.m)
Xc = (f1 + f2 + f3 + f4)*sin(gamma);
Zc = -(f1 + f2 + f3 + f4)*cos(gamma);
Lc = (d/sqrt(2))*(-f1 - f2 + f3 + f4);
Mc = (d/sqrt(2))*( f1 - f2 - f3 + f4);
Nc = km*(f1 - f2 + f3 - f4);

Fc = [Xc; 0; Zc];
Gc = [Lc; Mc; Nc];

% --- Aerodynamic drag forces and moments ---
Xaero = -nu * uB * abs(uB);
Yaero = -nu * vB * abs(vB);
Zaero = -nu * wB * abs(wB);

Laero = -mu * p * abs(p);
Maero = -mu * q * abs(q);
Naero = -mu * r * abs(r);

% --- Total forces and moments in body frame ---
X = Xc + Xaero;
Y = Yaero;
Z = Zc + Zaero;
L = Lc + Laero;
M = Mc + Maero;
N = Nc + Naero;

% --- Trig shorthands ---
cphi   = cos(phi);   sphi   = sin(phi);
ctheta = cos(theta); stheta = sin(theta);
cpsi   = cos(psi);   spsi   = sin(psi);
ttheta = tan(theta); stheta_sec = 1/ctheta; % sec(theta)

% --- Position kinematics (inertial frame) ---
xE_dot = (ctheta*cpsi)*uB + (sphi*stheta*cpsi - cphi*spsi)*vB + (cphi*stheta*cpsi + sphi*spsi)*wB;
yE_dot = (ctheta*spsi)*uB + (sphi*stheta*spsi + cphi*cpsi)*vB + (cphi*stheta*spsi - sphi*cpsi)*wB;
zE_dot = (-stheta)*uB    + (sphi*ctheta)*vB                   + (cphi*ctheta)*wB;

% --- Euler angle kinematics ---
phi_dot   = p + sphi*ttheta*q   + cphi*ttheta*r;
theta_dot =     cphi*q          - sphi*r;
psi_dot   =     sphi*stheta_sec*q + cphi*stheta_sec*r;

% --- Translational dynamics (body frame, Newton's 2nd law) ---
uB_dot = r*vB - q*wB + g*(-stheta)        + (1/m)*X;
vB_dot = p*wB - r*uB + g*(ctheta*sphi)    + (1/m)*Y;
wB_dot = q*uB - p*vB + g*(ctheta*cphi)    + (1/m)*Z;

% --- Rotational dynamics (Euler's equations, Ixz = 0 for symmetric body) ---
p_dot = (1/Ix)*(L + (Iy - Iz)*q*r);
q_dot = (1/Iy)*(M + (Iz - Ix)*p*r);
r_dot = (1/Iz)*(N + (Ix - Iy)*p*q);

% --- Assemble state derivative vector (12 states + Fc + Gc, same
% 18-element logging convention as QuadrotorEOM.m) ---
var_dot = [xE_dot; yE_dot; zE_dot; ...
           phi_dot; theta_dot; psi_dot; ...
           uB_dot; vB_dot; wB_dot; ...
           p_dot; q_dot; r_dot; Fc; Gc];

end
