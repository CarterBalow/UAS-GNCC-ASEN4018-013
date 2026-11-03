function [x, nis, ok] = eskf16_update_baro(x, zb, S)
% eskf16_update_baro - Baro height update:  z_b = -p_D + b_b + n_b
H = [0 0 -1, zeros(1,12), 1];
[x, nis, ok] = eskf16_update(x, zb - (-x.p(3) + x.bb), H, S.sigma_baro^2, S.gate1);
end
