function var_dot = ClosedLoopQuadrotorEOM(t, var, params, ref)
% ClosedLoopQuadrotorEOM - Wraps the nonlinear QuadrotorEOM with the
% ClosedLoopController (cruise-velocity outer loop + PID attitude inner
% loop) + motor mixing, so ode45 sees a self-contained closed-loop
% dynamics function. Feedback is recomputed from the current state at
% every integration step.
%
% var is the 21-element augmented state:
%   1:12  - real states [xE yE zE phi theta psi uB vB wB p q r]
%   13:18 - unused Fc/Gc slots, kept only for PlotAllStates compatibility
%           (overwritten in post-processing, not integrated meaningfully)
%   19:21 - integral states [xi_phi; xi_theta; xi_u] -- roll/pitch angle
%           error integrals (attitude PID) and forward-velocity error
%           integral (x-axis cruise PI) -- these ARE correctly integrated.

x12 = var(1:12);

[Fc, Gc, int_dot] = ClosedLoopController(var, params, ref);

motor_forces = ComputeMotorForces(Fc, Gc, params.d, params.km);

xdot12 = QuadrotorEOM(t, x12, params.g, params.m, params.I, ...
    params.d, params.km, params.nu, params.mu, motor_forces);

var_dot = [xdot12(1:12); zeros(6,1); int_dot];

end
