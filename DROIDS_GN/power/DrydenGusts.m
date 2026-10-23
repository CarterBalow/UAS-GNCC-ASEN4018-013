function [gust_NED, info] = DrydenGusts(t, W)
% DrydenGusts - Dryden turbulence time series (MIL-F-8785C, low altitude),
% rotated into NED. Returns the GUST only; add the mean wind separately.
%
% Low-altitude model (h < 1000 ft), computed in feet then converted:
%   L_w = h,   L_u = L_v = h / (0.177 + 0.000823 h)^1.2
%   sigma_w = 0.1*W20,   sigma_u = sigma_v = sigma_w / (0.177 + 0.000823 h)^0.4
% Shaping filters (V = mean airspeed through the turbulence, T = L/V):
%   H_u(s) = sigma_u sqrt(2 L_u/(pi V)) / (1 + T_u s)
%   H_v(s) = sigma_v sqrt(L_v/(pi V)) (1 + sqrt(3) T_v s) / (1 + T_v s)^2
%   H_w(s) = same form as H_v with L_w, sigma_w
% Driven by white noise of two-sided PSD pi, so each output's standard
% deviation equals its sigma. Discretized exactly (zero-order hold) at the
% sample time of t.
%
% Axes: u along the mean-wind direction, v 90 deg to its right, w down.
%
% Inputs:
%   t - uniformly spaced time vector (s)
%   W - struct: W20 (m/s, wind speed at 20 ft, sets intensity),
%               h (m, altitude AGL), V (m/s, airspeed through the gusts),
%               to_deg (deg, heading the mean wind blows TOWARD), seed
% Outputs:
%   gust_NED - Nx3 gust velocity (m/s)
%   info     - struct with sigma_u/v/w (m/s) and L_u/v/w (m)

ft = 0.3048;
h_ft = max(W.h, 0.5)/ft;
den  = 0.177 + 0.000823*h_ft;
L_w  = h_ft*ft;
L_u  = h_ft/den^1.2*ft;   L_v = L_u;
sig_w = 0.1*W.W20;
sig_u = sig_w/den^0.4;    sig_v = sig_u;
V  = max(W.V, 0.5);
dt = t(2) - t(1);
N  = numel(t);

% Repeatable random stream (MATLAB and Octave)
randn('seed', W.seed);
try, rng(W.seed); catch, end
n = sqrt(pi/dt)*randn(N, 3);   % white noise, two-sided PSD = pi

uvw = zeros(N, 3);
uvw(:,1) = FilterAxis(FirstOrderSS(sig_u, L_u, V),  n(:,1), dt);
uvw(:,2) = FilterAxis(SecondOrderSS(sig_v, L_v, V), n(:,2), dt);
uvw(:,3) = FilterAxis(SecondOrderSS(sig_w, L_w, V), n(:,3), dt);

chi = deg2rad(W.to_deg);
R = [cos(chi), -sin(chi), 0;
     sin(chi),  cos(chi), 0;
     0,         0,        1];
gust_NED = (R*uvw.').';

info = struct('sigma_u', sig_u, 'sigma_v', sig_v, 'sigma_w', sig_w, ...
              'L_u', L_u, 'L_v', L_v, 'L_w', L_w);
end

function sys = FirstOrderSS(sig, L, V)
T = L/V;  K = sig*sqrt(2*L/(pi*V));
sys.A = -1/T;  sys.B = K/T;  sys.C = 1;
end

function sys = SecondOrderSS(sig, L, V)
T = L/V;  K = sig*sqrt(L/(pi*V));
sys.A = [0, 1; -1/T^2, -2/T];
sys.B = [0; 1];
sys.C = (K/T^2)*[1, sqrt(3)*T];
end

function y = FilterAxis(sys, u, dt)
% Exact ZOH discretization via the augmented matrix exponential
n = size(sys.A, 1);
E = expm([sys.A, sys.B; zeros(1, n + 1)]*dt);
Ad = E(1:n, 1:n);  Bd = E(1:n, n+1);
x = zeros(n, 1);
y = zeros(numel(u), 1);
for k = 1:numel(u)
    y(k) = sys.C*x;
    x = Ad*x + Bd*u(k);
end
end
