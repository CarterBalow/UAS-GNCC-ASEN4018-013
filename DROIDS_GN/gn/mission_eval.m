function [p_ned_m, v_ned_mps, a_ned_mps2, psi_rad, psi_dot_rps, done] = mission_eval(mission, stage, t_stage_s) %#codegen
%MISSION_EVAL  Reference p, v, a, yaw for a stage at time t_stage_s since that stage began.
%   stage: 0 idle on ground, 1 takeoff, 2 transit, 3 search, 4 return, 5 landing, 6 landed
%   done: stage trajectory finished (holding its end point at rest).

    p_ned_m = mission.home_ned_m;
    v_ned_mps = zeros(3, 1);
    a_ned_mps2 = zeros(3, 1);
    psi_rad = mission.psi;
    psi_dot_rps = 0;
    done = true;
    switch stage
        case 1
            [p_ned_m, v_ned_mps, a_ned_mps2, psi_rad, psi_dot_rps] = traj_eval(mission.takeoff, t_stage_s);
        case 2
            [p_ned_m, v_ned_mps, a_ned_mps2, psi_rad, psi_dot_rps] = traj_eval(mission.transit, t_stage_s);
        case 3
            [p_ned_m, v_ned_mps, a_ned_mps2, psi_rad, psi_dot_rps] = traj_eval(mission.search, t_stage_s);
        case 4
            [p_ned_m, v_ned_mps, a_ned_mps2, psi_rad, psi_dot_rps] = traj_eval(mission.return_home, t_stage_s);
        case 5
            [p_ned_m, v_ned_mps, a_ned_mps2, psi_rad, psi_dot_rps] = traj_eval(mission.landing, t_stage_s);
    end
    if stage >= 1 && stage <= 5
        done = t_stage_s >= mission.T(stage);
    end
end
