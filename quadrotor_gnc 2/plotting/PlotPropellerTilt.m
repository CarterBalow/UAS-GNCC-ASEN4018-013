function PlotPropellerTilt(t, gamma_hist, gamma_dot_hist, case_title)
% PlotPropellerTilt - Plots the propeller-tilt actuator state (gamma) and
% its rate over time, for the thrust-vectoring design.
%
% Inputs:
%   t               - time vector (Nx1)
%   gamma_hist      - Nx1 propeller tilt angle (rad)
%   gamma_dot_hist  - Nx1 propeller tilt rate (rad/s)
%   case_title      - string label for the figure title (optional)

if nargin < 4
    case_title = 'Propeller Tilt (Thrust Vectoring)';
end

figure('Name', case_title, 'NumberTitle', 'off');

subplot(2,1,1);
plot(t, rad2deg(gamma_hist), 'b', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('\gamma (deg)');
title('Propeller Tilt Angle'); grid on;

subplot(2,1,2);
plot(t, rad2deg(gamma_dot_hist), 'r', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('\gamma dot (deg/s)');
title('Propeller Tilt Rate'); grid on;

try
    sgtitle(case_title);
catch
    % sgtitle not available (e.g. Octave) -- subplot titles are enough
end

end
