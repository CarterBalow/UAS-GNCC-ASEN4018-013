function [x, info] = eskf16_update_gps(x, zp, zv, S)
% eskf16_update_gps - GPS position, then velocity, update (NED). Pass [] to skip one.
info = struct('nis_p', NaN, 'ok_p', true, 'nis_v', NaN, 'ok_v', true);
if ~isempty(zp)
    H = [eye(3), zeros(3,13)];
    [x, info.nis_p, info.ok_p] = eskf16_update(x, zp(:) - x.p, H, diag(S.sigma_gps_p.^2), S.gate3);
end
if ~isempty(zv)
    H = [zeros(3), eye(3), zeros(3,10)];
    [x, info.nis_v, info.ok_v] = eskf16_update(x, zv(:) - x.v, H, diag(S.sigma_gps_v.^2), S.gate3);
end
end
