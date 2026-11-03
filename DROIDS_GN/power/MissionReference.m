function [p_ref, v_ref, a_ref] = MissionReference(M, t)
% MissionReference - Mission position, velocity and acceleration (NED) at
% time t, by linear interpolation of the uniformly sampled mission profile.
%
% Inputs:
%   M - mission struct from main_motorf_robust (fields t, pos, vel, acc; Nx1, Nx3)
%   t - time (s); held at the last sample after the mission ends
%
% Outputs:
%   p_ref, v_ref, a_ref - 3x1 position (m), velocity (m/s), acceleration
%                         (m/s^2), NED. a_ref is zero outside the mission.

dt = M.t(2) - M.t(1);
u  = (t - M.t(1))/dt;
k  = floor(u) + 1;
if k < 1
    p_ref = M.pos(1,:).';  v_ref = M.vel(1,:).';  a_ref = zeros(3,1);  return
elseif k >= numel(M.t)
    p_ref = M.pos(end,:).';  v_ref = M.vel(end,:).';  a_ref = zeros(3,1);  return
end
a = u - (k - 1);
p_ref = ((1 - a)*M.pos(k,:) + a*M.pos(k+1,:)).';
v_ref = ((1 - a)*M.vel(k,:) + a*M.vel(k+1,:)).';
a_ref = ((1 - a)*M.acc(k,:) + a*M.acc(k+1,:)).';
end
