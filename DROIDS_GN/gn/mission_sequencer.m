function [stage, t_stage_s] = mission_sequencer(mission, t_s, go, p_ned_m) %#codegen
%MISSION_SEQUENCER  Flight-stage state machine: idle -> takeoff -> transit -> search -> return -> landing -> landed.
%   go starts the mission from idle. A stage ends when its trajectory is done and the
%   vehicle position p_ned_m is within tol_m of the stage end point.
%   t_stage_s: time since the current stage began (feed to mission_eval).

    tol_m = 0.15;
    persistent s t_enter
    if isempty(s)
        s = 0;
        t_enter = t_s;
    end
    if s == 0
        if go
            s = 1;
            t_enter = t_s;
        end
    elseif s <= 5
        if t_s - t_enter >= mission.T(s) && norm(p_ned_m - mission.p_end(:, s)) <= tol_m
            s = s + 1;
            t_enter = t_s;
        end
    end
    stage = s;
    t_stage_s = t_s - t_enter;
end
