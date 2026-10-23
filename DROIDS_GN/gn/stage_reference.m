function [pRef, vRef, aRef, stageDone] = stage_reference(mission, phase, t_s) %#codegen
%STAGE_REFERENCE  Smoothed p, v, a for the Mission Manager phase (0-6), timed from phase entry.
%   stageDone: the phase trajectory has finished (use as searchDone in phase 3).

    persistent last t_enter
    if isempty(last)
        last = -1;
        t_enter = 0;
    end
    if phase ~= last
        last = phase;
        t_enter = t_s;
    end
    [pRef, vRef, aRef, ~, ~, stageDone] = mission_eval(mission, phase, t_s - t_enter);
end
