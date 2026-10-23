function motor_forces = ComputeMotorForces(Fc, Gc, d, km)
% ComputeMotorForces - Solves for individual motor forces f1-f4
%
% Inputs:
%   Fc  - Control force vector  [Xc, Yc, Zc]  (1x3 or 3x1)
%   Gc  - Control moment vector [Lc, Mc, Nc]  (1x3 or 3x1)
%   d   - Arm length (m)
%   km  - Motor torque constant
%
% Output:
%   motor_forces - [f1; f2; f3; f4] individual motor thrust forces (N)

% Extract relevant components (works for either 1x3 or 3x1 inputs)
Zc = Fc(3);    % Thrust (negative = upward in body frame)
Lc = Gc(1);    % Roll moment
Mc = Gc(2);    % Pitch moment
Nc = Gc(3);    % Yaw moment

% Build the mixing matrix A such that A * [f1;f2;f3;f4] = [Zc;Lc;Mc;Nc]
%
%   Zc = -(f1 + f2 + f3 + f4)
%   Lc = (d/sqrt(2))*(-f1 - f2 + f3 + f4)
%   Mc = (d/sqrt(2))*( f1 - f2 - f3 + f4)
%   Nc = km*(f1 - f2 + f3 - f4)

s = d / sqrt(2);

A = [ -1,  -1,  -1,  -1;
      -s,  -s,   s,   s;
       s,  -s,  -s,   s;
      km,  -km,  km, -km ];

% Right-hand side vector (column, to match A*f = b)
b = [Zc; Lc; Mc; Nc];

% Solve the linear system: A * f = b
% NOTE: previously this was `(A/b)'` with b as a row vector, which does
% NOT solve A*f=b (mrdivide against a 1x4 "matrix" silently returns
% garbage -- e.g. hover input [Zc,0,0,0] produced motor forces
% [1.499, 0.064, -0.064, -0.0036] instead of four equal ~0.167 N
% forces). Fixed to a plain backslash solve.
motor_forces = A \ b;

end
