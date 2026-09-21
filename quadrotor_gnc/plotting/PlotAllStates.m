function PlotAllStates(t, state, case_title)
% PlotAllStates - Plots every state vector value over time
%
% Inputs:
%   t           - time vector (Nx1)
%   state       - state matrix (Nx18)
%                 Cols 1-3:   [xE, yE, zE]         inertial position (m)
%                 Cols 4-6:   [phi, theta, psi]     Euler angles (rad)
%                 Cols 7-9:   [uB, vB, wB]          body-frame velocity (m/s)
%                 Cols 10-12: [p, q, r]             angular rates (rad/s)
%                 Cols 13-15: [Fc_x, Fc_y, Fc_z]   control forces (N)
%                 Cols 16-18: [Gc_x, Gc_y, Gc_z]   control moments (N*m)
%   case_title  - string label for the figure title (optional)

if nargin < 3
    case_title = 'All State Variables';
end

% --- Extract states ---
xE    = state(:,1);
yE    = state(:,2);
zE    = state(:,3);
phi   = state(:,4);
theta = state(:,5);
psi   = state(:,6);
uB    = state(:,7);
vB    = state(:,8);
wB    = state(:,9);
p     = state(:,10);
q     = state(:,11);
r     = state(:,12);

% Control forces / moments (if present)
has_control = size(state, 2) >= 18;
if has_control
    Fc_x = state(:,13);
    Fc_y = state(:,14);
    Fc_z = state(:,15);
    Gc_x = state(:,16);
    Gc_y = state(:,17);
    Gc_z = state(:,18);
end

% Convert angles and rates to degrees
phi_deg   = rad2deg(phi);
theta_deg = rad2deg(theta);
psi_deg   = rad2deg(psi);
p_deg     = rad2deg(p);
q_deg     = rad2deg(q);
r_deg     = rad2deg(r);

% --- Figure setup ---
n_rows = 4 + 2*has_control;  % 4 rows always, +2 if control states exist
figure('Name', case_title, 'NumberTitle', 'off', ...
       'Units', 'normalized', 'OuterPosition', [0 0 1 1]);
sgtitle(case_title, 'FontSize', 14, 'FontWeight', 'bold');

% --- Row 1: Inertial Position ---
subplot(n_rows, 3, 1);
plot(t, xE, 'b', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('x_E (m)'); title('Inertial Position X'); grid on;

subplot(n_rows, 3, 2);
plot(t, yE, 'r', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('y_E (m)'); title('Inertial Position Y'); grid on;

subplot(n_rows, 3, 3);
plot(t, zE, 'g', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('z_E (m)'); title('Inertial Position Z'); grid on;

% --- Row 2: Euler Angles ---
subplot(n_rows, 3, 4);
plot(t, phi_deg, 'b', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('\phi (deg)'); title('Roll Angle \phi'); grid on;

subplot(n_rows, 3, 5);
plot(t, theta_deg, 'r', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('\theta (deg)'); title('Pitch Angle \theta'); grid on;

subplot(n_rows, 3, 6);
plot(t, psi_deg, 'g', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('\psi (deg)'); title('Yaw Angle \psi'); grid on;

% --- Row 3: Body-Frame Velocities ---
subplot(n_rows, 3, 7);
plot(t, uB, 'b', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('u_B (m/s)'); title('Body Velocity u_B'); grid on;

subplot(n_rows, 3, 8);
plot(t, vB, 'r', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('v_B (m/s)'); title('Body Velocity v_B'); grid on;

subplot(n_rows, 3, 9);
plot(t, wB, 'g', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('w_B (m/s)'); title('Body Velocity w_B'); grid on;

% --- Row 4: Angular Rates ---
subplot(n_rows, 3, 10);
plot(t, p_deg, 'b', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('p (deg/s)'); title('Roll Rate p'); grid on;

subplot(n_rows, 3, 11);
plot(t, q_deg, 'r', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('q (deg/s)'); title('Pitch Rate q'); grid on;

subplot(n_rows, 3, 12);
plot(t, r_deg, 'g', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('r (deg/s)'); title('Yaw Rate r'); grid on;

% --- Rows 5-6: Control Forces & Moments (if present) ---
if has_control
    subplot(n_rows, 3, 13);
    plot(t, Fc_x, 'b', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('F_{c,x} (N)'); title('Control Force F_{c,x}'); grid on;

    subplot(n_rows, 3, 14);
    plot(t, Fc_y, 'r', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('F_{c,y} (N)'); title('Control Force F_{c,y}'); grid on;

    subplot(n_rows, 3, 15);
    plot(t, Fc_z, 'g', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('F_{c,z} (N)'); title('Control Force F_{c,z}'); grid on;

    subplot(n_rows, 3, 16);
    plot(t, Gc_x, 'b', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('G_{c,x} (N{\cdot}m)'); title('Control Moment G_{c,x}'); grid on;

    subplot(n_rows, 3, 17);
    plot(t, Gc_y, 'r', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('G_{c,y} (N{\cdot}m)'); title('Control Moment G_{c,y}'); grid on;

    subplot(n_rows, 3, 18);
    plot(t, Gc_z, 'g', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('G_{c,z} (N{\cdot}m)'); title('Control Moment G_{c,z}'); grid on;
end

end
