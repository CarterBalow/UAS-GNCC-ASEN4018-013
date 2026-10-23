function [x_dot, info] = RobustClosedLoopEOM(t, x, P, ref0, M, wind_NED, meas_noise)
% RobustClosedLoopEOM - Closed-loop quadrotor flying the mission profile:
%
%   mission p,v(t) -> velocity command -> ClosedLoopController (your
%   body-tilt cascade, unchanged) -> ComputeMotorForces (mixer) ->
%   saturation [f_min, f_max] -> first-order motor lag -> RobustQuadrotorEOM
%   (wind/gust drag + CP moment)
%
% The controller sees a MEASURED state (true state + sensor noise); the
% plant integrates the TRUE state.
%
% State x (27 elements):
%   1:12   real states [xE yE zE phi theta psi uB vB wB p q r]
%   13:18  unused (kept so the 1:23 layout matches ClosedLoopController)
%   19:23  controller integrals [xi_phi xi_theta xi_u xi_v xi_z]
%   24:27  actual motor thrusts [f1 f2 f3 f4] (N)
%
% Velocity command per axis (mission tracking, inertial NED):
%   v_cmd = v_mis(t) + kpos*(p_mis(t) - p_meas)
% i.e. the mission velocity as feedforward plus your position-loop gain
% pulling the vehicle back onto the mission path. It is passed to
% ClosedLoopController in 'velocity' mode, so the PI velocity loop, tilt
% inversion, thrust law and attitude PID are exactly your existing ones.
%
% Inputs:
%   ref0       - ref struct: body-tilt gains (GB.*) + caps + mode fields
%   M          - mission struct (t, pos, vel)
%   wind_NED   - wind (mean + gust) at this step, 3x1 (m/s), held over the step
%   meas_noise - 12x1 sensor noise added to x(1:12) for the controller
%
% Outputs:
%   x_dot - state derivative
%   info  - (optional) struct with commanded/actual motor forces, Fc, Gc, refs

x12 = x(1:12);
f_act = x(24:27);

% --- Measured state for the controller ---
xm = x(1:23);
xm(1:12) = x12 + meas_noise(:);

% --- Mission tracking -> velocity commands ---
[p_ref, v_ref] = MissionReference(M, t);
ref = ref0;
ref.vx = v_ref(1) + ref0.kpx_pos*(p_ref(1) - xm(1));
ref.vy = v_ref(2) + ref0.kpy_pos*(p_ref(2) - xm(2));
ref.vz = v_ref(3) + ref0.kpz_pos*(p_ref(3) - xm(3));

% --- Your controller and mixer (unchanged, from quadrotor_gnc 2) ---
[Fc, Gc, int_dot, dbg] = ClosedLoopController(xm, P, ref);
f_cmd = ComputeMotorForces(Fc, Gc, P.d, P.km);

% --- Motor saturation + first-order lag:  tau_m*f_dot + f = sat(f_cmd) ---
f_sat = min(max(f_cmd, P.f_min), P.f_max);
f_dot = (f_sat - f_act)/P.tau_m;

% --- Plant ---
x12_dot = RobustQuadrotorEOM(x12, P, f_act, wind_NED);

x_dot = [x12_dot; zeros(6,1); int_dot(:); f_dot];

if nargout > 1
    info = struct('f_cmd', f_cmd, 'f_act', f_act, 'Fc', Fc, 'Gc', Gc, ...
                  'p_ref', p_ref, 'v_ref', v_ref, ...
                  'phi_ref', dbg.phi_ref, 'theta_ref', dbg.theta_ref);
end
end
