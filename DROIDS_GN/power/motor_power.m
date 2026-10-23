function P_W = motor_power(f_N, f_hover_N, P_hover_W, n) %#codegen
%MOTOR_POWER  Electrical power per motor from thrust, P = P_hover*(f/f_hover)^n (fit to the weight/power sheet).
    P_W = P_hover_W * (max(f_N, 0) / f_hover_N).^n;
end
