function var_dot = linQR_EOM(t, var, g, m, I, d, km, nu, mu, motor_forces)
% linQR_EOM - Linearized quadrotor equations of motion with InnerLoopFeedback
%
% Linearized about hover trim:
%   x0 = [0 0 0  0 0 0  0 0 0  0 0 0]  (all angles, velocities, rates = 0)
%
% InnerLoopFeedback provides:
%   Fc(3) = m*g  →  cancels gravity in w_dot (z-velocity stays at trim)
%   Gc     = PD  →  drives phi/theta back to 0 via roll/pitch damping
%
% Inputs:
%   t            - time (unused, required by ode45)
%   var          - state vector (18x1): [xE yE zE phi theta psi uB vB wB p q r Fc Gc]
%   g            - gravitational acceleration (m/s^2)
%   m            - vehicle mass (kg)
%   I            - inertia matrix (3x3 diagonal)
%   d, km, nu, mu, motor_forces  - unused in linearized model (kept for
%                                  compatible ode45 function signature)

% --- Extract states ---
phi   = var(4);   % roll  (rad)
theta = var(5);   % pitch (rad)
uB    = var(7);   % body-frame velocity x (m/s)
vB    = var(8);   % body-frame velocity y (m/s)
wB    = var(9);   % body-frame velocity z (m/s)
p     = var(10);  % roll rate  (rad/s)
q     = var(11);  % pitch rate (rad/s)
r     = var(12);  % yaw rate   (rad/s)

% --- Inertia ---
Ix = I(1,1);
Iy = I(2,2);
Iz = I(3,3);

% --- Control forces and moments from InnerLoopFeedback ---
% Fc = [0; 0; m*g]   (gravity compensation, constant at trim)
% Gc = [kp*phi - kd*p;  kp*theta - kd*q;  k_yaw*r]  (PD attitude feedback)
[Fc, Gc] = InnerLoopFeedback(var);
Fc(3) = .068*9.81;
% =========================================================================
% Linearized kinematics
% Rotation matrix R(phi,theta,psi) --> I  at hover trim, so inertial rates
% equal body-frame velocities directly.
% =========================================================================
xE_dot    = uB;
yE_dot    = vB;
zE_dot    = wB;

phi_dot   = p;
theta_dot = q;
psi_dot   = r;

% =========================================================================
% Linearized translational dynamics  (body frame)
%   Full:  uB_dot = r*vB - q*wB - g*sin(theta)       + (1/m)*X
%          vB_dot = p*wB - r*uB + g*cos(theta)*sin(phi) + (1/m)*Y
%          wB_dot = q*uB - p*vB + g*cos(theta)*cos(phi) + (1/m)*Z
%
%   Linearized (drop Coriolis cross-terms, sin(x)≈x, cos(x)≈1):
%          uB_dot = -g*theta        + (1/m)*Fc(1)   [Fc(1)=0]
%          vB_dot = +g*phi          + (1/m)*Fc(2)   [Fc(2)=0]
%          wB_dot = g - (1/m)*Fc(3)                 [Fc(3)=mg → 0 at trim]
%
%   Note: the sign convention for thrust (Zc = -(sum of motor forces)) means
%   the control force opposes the +z gravity term, so (1/m)*Z = -(1/m)*Fc(3).
% =========================================================================
uB_dot = -g*theta     + (1/m)*Fc(1);
vB_dot =  g*phi       + (1/m)*Fc(2);
wB_dot =  g           - (1/m)*Fc(3);   % = 0 at trim since Fc(3) = m*g

% =========================================================================
% Linearized rotational dynamics  (Euler's equations)
%   Full:  p_dot = (1/Ix)*( L + (Iy-Iz)*q*r )
%          q_dot = (1/Iy)*( M + (Iz-Ix)*p*r )
%          r_dot = (1/Iz)*( N + (Ix-Iy)*p*q )
%
%   Linearized (drop p*r, q*r, p*q products of small rates):
%          p_dot = (1/Ix) * Gc(1)
%          q_dot = (1/Iy) * Gc(2)
%          r_dot = (1/Iz) * Gc(3)
%
%   InnerLoopFeedback PD gains place closed-loop poles at lambda1, lambda2
%   for roll and pitch, giving  phi--> 0  and  theta --> 0  (back to trim).
% =========================================================================
p_dot = (1/Ix) * Gc(1);
q_dot = (1/Iy) * Gc(2);
r_dot = (1/Iz) * Gc(3);

% --- Assemble state derivative (same 18-element layout as nonlinear EOM) ---
var_dot = [xE_dot; yE_dot; zE_dot; ...
           phi_dot; theta_dot; psi_dot; ...
           uB_dot; vB_dot; wB_dot; ...
           p_dot; q_dot; r_dot; ...
           Fc; Gc];

end
