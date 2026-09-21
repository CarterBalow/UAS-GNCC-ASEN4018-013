function var_dot = closedloopEOMthrustvec(t, var, params, ref)
% closedloopEOMthrustvec - Wraps the thrust-vectoring nonlinear EOM
% (quadrotor_thrustvec) with the thrust-vectoring controller
% (closedloopthrustvec) + motor mixing, so ode45 sees a self-contained
% closed-loop dynamics function.
%
% IMPORTANT: calls quadrotor_thrustvec(...) and closedloopthrustvec(...)
% by their actual filenames, not "QuadrotorEOM"/"ClosedLoopController" --
% MATLAB/Octave dispatch a function call by the FILE on the path, so
% calling those names here would silently run the ORIGINAL body-tilt
% files instead of these thrust-vectoring ones (both live in the same
% folder). This was the biggest risk in the original draft: it would
% have run without error but quietly simulated the wrong design.
%
% var is the 24-element augmented state:
%   1:12  - real states [xE yE zE phi theta psi uB vB wB p q r]
%   13:18 - unused Fc/Gc slots, kept only for PlotAllStates compatibility
%   19:21 - [xi_phi; xi_theta(unused); xi_u] -- roll PID integral and
%           x-cruise PI integral (xi_theta kept only to preserve layout)
%   22:24 - [gamma; gamma_dot; gammai] -- propeller-tilt actuator state
%           and its error integral, all correctly integrated by ode45

x12 = var(1:12);
gamma = var(22);
gamma_dot = var(23);

[Fc, Gc, int_dot, gamma_ddot, gamma_err] = closedloopthrustvec(var, params, ref);

motor_forces = ComputeMotorForces(Fc, Gc, params.d, params.km);

xdot12 = quadrotor_thrustvec(t, x12, params.g, params.m, params.I, ...
    params.d, params.km, params.nu, params.mu, motor_forces, gamma);

var_dot = [xdot12(1:12); zeros(6,1); int_dot; gamma_dot; gamma_ddot; gamma_err];

end
