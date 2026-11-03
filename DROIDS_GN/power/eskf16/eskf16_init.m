function x = eskf16_init(S, abar, wbar, p0, psi0, use_baro)
% eskf16_init - Nominal state + covariance from a still window on the ground.
%   abar, wbar : mean accelerometer / gyro over the window (body FRD)
%   p0         : initial NED position (N, E from GPS; D from baro or GPS)
%   psi0       : assumed heading (no magnetometer)
%   use_baro   : true -> baro is the height reference (tight p_D, b_b = 0)
phi0   = atan2(-abar(2), -abar(3));                      % tilt from gravity
theta0 = atan2( abar(1), sqrt(abar(2)^2 + abar(3)^2));
x.q  = eskf16_dcm2quat(eskf16_euler2dcm([phi0; theta0; psi0]));
x.bg = wbar(:);                                          % at rest: gyro mean = bias
x.ba = zeros(3,1);
x.bb = 0;                                                % baro zeroed at arm
x.v  = zeros(3,1);                                       % on the ground, not moving
x.p  = p0(:);
sig_pD0 = 0.2;  if ~use_baro, sig_pD0 = S.sigma_gps_p(3); end
x.P = diag([ S.sigma_gps_p(1:2).^2; sig_pD0^2; ...
             0.05^2*ones(3,1); ...
             deg2rad([2; 2; S.yaw0_sigma_deg]).^2; ...
             deg2rad(0.1)^2*ones(3,1); ...
             0.2^2*ones(3,1); ...
             0.1^2 ]);
end
