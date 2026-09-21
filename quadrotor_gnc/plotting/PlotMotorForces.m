function PlotMotorForces(t, motor_forces_hist, case_title)
% PlotMotorForces - Plots each individual motor's thrust force over time.
%
% Inputs:
%   t                  - time vector (Nx1)
%   motor_forces_hist  - Nx4 matrix of motor forces [f1 f2 f3 f4] (N)
%   case_title         - string label for the figure title (optional)

if nargin < 3
    case_title = 'Motor Thrust Forces';
end

f1 = motor_forces_hist(:,1);
f2 = motor_forces_hist(:,2);
f3 = motor_forces_hist(:,3);
f4 = motor_forces_hist(:,4);

figure('Name', case_title, 'NumberTitle', 'off');
plot(t, f1, 'b', 'LineWidth', 1.5); hold on;
plot(t, f2, 'r', 'LineWidth', 1.5);
plot(t, f3, 'g', 'LineWidth', 1.5);
plot(t, f4, 'm', 'LineWidth', 1.5);
hold off;

xlabel('Time (s)');
ylabel('Motor Force (N)');
title(case_title);
legend('f_1', 'f_2', 'f_3', 'f_4', 'Location', 'best');
grid on;

end
