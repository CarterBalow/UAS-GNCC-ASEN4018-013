function S = eskf16_specs(board, gps_mode)
% eskf16_specs - Noise / bias parameters for each sensor (1-sigma unless noted).
S.board = lower(board);  S.gps_mode = lower(gps_mode);
switch S.board
    case 'pixhawk6c'
        % --- IMU: TDK ICM-42688-P (datasheet DS-000347, low-noise mode) ---
        %   gyro 2.8 mdps/sqrt(Hz), accel 70 ug/sqrt(Hz) (z; 65 x/y)
        %   x10 for airframe vibration in flight (typical; tune from a flight log)
        S.sigma_g  = 10*deg2rad(2.8e-3);      % rad/s/sqrt(Hz)
        S.sigma_a  = 10*70e-6*9.80665;        % m/s^2/sqrt(Hz)
        % bias random walk: not on the datasheet -> ASSUMED until an Allan
        % variance from a stationary Pixhawk log replaces it
        S.sigma_bg = 1e-5;                    % rad/s^2/sqrt(Hz)
        S.sigma_ba = 1e-4;                    % m/s^3/sqrt(Hz)

        % --- Baro: TE MS5611-01BA03 ---
        %   resolution 0.012 mbar RMS at OSR 4096; at Boulder (rho ~1.03 kg/m^3)
        %   dh = dp/(rho g) -> 1.2 Pa / (1.03*9.81) = 0.12 m. x3 for prop wash /
        %   dynamic pressure on the board (ASSUMED).
        %   Absolute accuracy +/-1.5 mbar (~15 m) and drift (+/-1 mbar/yr, weather,
        %   self-heating) are why b_b is a state: it's zeroed at arm, then walks.
        S.sigma_baro = 3*(0.012*100)/(1.03*9.81);   % m  (~0.36 m)
        S.sigma_bb   = 0.02;                         % m/sqrt(s)  bias random walk (ASSUMED)

    case 'pixhawk6x'
        error(['sensor_specs: Pixhawk 6X specs not entered yet. Copy the pixhawk6c ', ...
               'block and replace the IMU noise densities (gyro, accel) and the baro ', ...
               'resolution with the 6X parts (check the exact board revision).']);
    otherwise
        error('sensor_specs: unknown board "%s"', board);
end

% --- GNSS: Holybro H-RTK F9P Helical (u-blox F9P) ---
%   datasheet -> 1-sigma: horizontal CEP = 1.1774 sigma, vertical R50 = 0.6745 sigma
%   PVT: 1.5 m CEP / 2.0 m R50 -> 1.27 m / 2.97 m
%   RTK: 0.01 m + 1 ppm        -> ~0.0085 m / ~0.015 m, x2 margin
%   velocity 0.05 m/s (50%); vertical velocity not given (ASSUMED 0.10)
%   NOTE: real standalone error is time-correlated (wanders over minutes);
%   white noise at these sigmas is optimistic about averaging it out.
switch S.gps_mode
    case 'rtk', S.sigma_gps_p = [0.017; 0.017; 0.03];
    case 'pvt', S.sigma_gps_p = [1.27; 1.27; 2.97];
    otherwise,  error('sensor_specs: gps_mode must be ''pvt'' or ''rtk''');
end
S.sigma_gps_v = [0.05; 0.05; 0.10];

S.gate3 = 11.34;           % chi^2, 3 dof, 99%
S.gate1 = 6.63;            % chi^2, 1 dof, 99%
S.yaw0_sigma_deg = 10;     % initial heading uncertainty (no magnetometer)
end
