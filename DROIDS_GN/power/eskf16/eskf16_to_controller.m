function x12 = eskf16_to_controller(x, w_body)
% eskf16_to_controller - ESKF state -> ClosedLoopController 12-state
%   [xE yE zE phi theta psi uB vB wB p q r]
%   w_body : body rate to hand the controller (bias-corrected, optionally low-passed)
Q   = eskf16_q2dcm(x.q);
x12 = [x.p; eskf16_dcm2euler(Q); Q.'*x.v; w_body(:)];
end
