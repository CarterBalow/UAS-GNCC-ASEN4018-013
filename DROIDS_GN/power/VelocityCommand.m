function v_cmd = VelocityCommand(mode, ref, pos_field, vel_field, k_pos, pos, v_max)
% VelocityCommand - outer-loop velocity command for one axis, by target
% mode. Shared by ClosedLoopController (body tilt) and closedloopthrustvec
% (thrust vectoring).
%   'position': v_cmd = sat(k_pos*(ref.<pos_field> - pos), v_max)
%   'velocity': v_cmd = sat(ref.<vel_field>, v_max)
switch lower(mode)
    case 'position'
        v_cmd = k_pos*(ref.(pos_field) - pos);
    case 'velocity'
        v_cmd = ref.(vel_field);
    otherwise
        error('VelocityCommand: ref.%s_mode must be ''position'' or ''velocity'' (got ''%s'')', ...
              pos_field, mode);
end
v_cmd = max(min(v_cmd, v_max), -v_max);
end
