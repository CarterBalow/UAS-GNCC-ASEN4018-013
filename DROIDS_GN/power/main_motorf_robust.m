function results = main_motorf_robust(cfg)
%% main_motorf_robust - Per-motor thrust over the mission, from a closed-loop
%% model with disturbances (instead of the open-loop inverse dynamics in
%% MissionMotorForces.m).
%%
%% Model:
%%   mission p,v(t) -> velocity command -> ClosedLoopController (body tilt,
%%   your existing cascade + gains from run_all_designs.m) -> mixer ->
%%   motor saturation + first-order lag -> nonlinear EOM with
%%   wind/gust drag and center-of-pressure moment -> noisy sensors -> back
%%   to the controller.
%%
%% Several cases are run back to back on the SAME mission so you can see
%% what each effect adds to the motor forces (e.g. ideal vs. wind+gusts vs.
%% wind+gusts+sensor noise). Edit everything in the SETTINGS block.
%%
%% Mission comes from the DROIDS_GN planner (gn_init cfg -> gn_build_mission),
%% and the motor forces are turned into electrical power with the 3.2 kg row
%% of Weight and Power.xlsx (Sheet3).
%%
%% Files in this folder:
%%   main_motorf_robust.m   - this script (settings, sim loop, metrics, plots)
%%   RobustClosedLoopEOM.m  - mission tracking + controller + mixer + motors + plant
%%   RobustQuadrotorEOM.m   - 12-state EOM with air-relative drag and CP moment
%%   DrydenGusts.m          - MIL-F-8785C Dryden turbulence, low altitude
%%   MissionReference.m     - mission position/velocity at time t
%%   PlotMotorfRobust.m     - figures
%%   PlotMissionPower.m     - power / energy figure
%%   motor_power.m          - per-motor power from thrust (fit to the sheet)
%%   Weight and Power.xlsx  - weight / power budget
%% Outputs go to power/figures/.
%%
%% Call with no input to use gn_init's cfg, or pass cfg (e.g. from
%% test_search_boustrophedon) to fly that mission.

this_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(fileparts(this_dir), 'gn'));   % planner (gn_init, gn_build_mission)
if nargin < 1
    gn_init;                                    % -> cfg
end
addpath(this_dir);                              % this folder first on the path

%% ============================ SETTINGS =================================

%% --- Vehicle + body-tilt gains (run_all_designs.m SETTINGS) ---
P.g  = 9.81;
P.m  = 3.2;          % kg, AUW (Weight and Power.xlsx row used below)
P.Ix = .0045813;     % inertias still from the lighter run_all_designs vehicle
P.Iy = .0045833;
P.Iz = .0080147;
P.I  = diag([P.Ix; P.Iy; P.Iz]);
P.d  = .18785;       % arm length (m)
P.km = 0.0045;       % motor torque constant
P.mu = 2e-6;

GB.u_max   = .5;
GB.kpx_pos = 0.635;
GB.kpu = 1.153;  GB.kiu = 0.05969;
GB.v_max   = .5;
GB.kpy_pos = 0.635;
GB.kpv = 1.153;  GB.kiv = 0.05969;
GB.w_max   = .5;
GB.kpz_pos = 4.441;
GB.kpw = 9.992;  GB.kiw = 2.064;

%% --- Power: Weight and Power.xlsx, Sheet3 ---
PW.xlsx = fullfile(this_dir, 'Weight and Power.xlsx');
PW.AUW  = P.m;       % kg, row of the sheet to use

%% --- Motor: EMAX ECO III (E3) 2807 1300KV, AVIA 7035 3-blade, 6S ---
P.f_max = 22.4;      % N, full-throttle thrust (2290 g)
P.f_min = 0;         % N, motors can't push down
P.kT    = 3.27e-6;   % N/(rad/s)^2, fit of f = kT*Omega^2 to the thrust table
P.tau_m = 0.04;      % s, motor+prop spin-up time constant (ASSUMED, 7" prop)

%% --- Aerodynamics ---
AERO.use_run_all_drag = false;  % true: keep nu from run_all_designs.m
AERO.rho  = 1.03;    % kg/m^3, air density at ~1650 m (Boulder)
AERO.Cd   = 1.0;     % body drag coefficient (ASSUMED, bluff frame)
AERO.area = 0.04;    % m^2, frontal area (ASSUMED -- measure from CAD)
P.r_cp    = [0; 0; -0.02];   % m, center of pressure rel. to CG, body FRD
                             % (ASSUMED: 2 cm above CG; drag x r_cp = moment)
if ~AERO.use_run_all_drag
    P.nu = 0.5*AERO.rho*AERO.Cd*AERO.area;   % N/(m/s)^2
else
    P.nu = 1.0e-3;
end

%% --- Wind (applies to cases with wind = true) ---
W.speed  = 3;        % m/s, mean wind speed
W.to_deg = 90;       % deg, heading the wind blows TOWARD (90 = blowing east)
W.W20    = 5;        % m/s, Dryden intensity parameter (wind at 20 ft):
                     %   ~7.7 light, ~15 moderate, ~23 severe (MIL-F-8785C)
W.h      = cfg.altitude;   % m, flight altitude AGL (mission height)
W.seed   = 1;        % random seed (repeatable gusts)

%% --- Sensor / estimate noise (applies to cases with noise = true) ---
% Each channel is first-order colored noise: 1-sigma value, plus a
% correlation time ~ 1/bandwidth of that signal coming out of the
% estimator/filters (white 250 Hz noise on position would be unrealistic
% and would be amplified straight into thrust by the z gains).
NS.pos  = 0.05;          NS.tau_pos  = 0.5;    % m,     s  (position estimate)
NS.att  = deg2rad(0.3);  NS.tau_att  = 0.1;    % rad,   s  (attitude estimate)
NS.vel  = 0.05;          NS.tau_vel  = 0.2;    % m/s,   s  (velocity estimate)
NS.rate = deg2rad(0.5);  NS.tau_rate = 0.01;   % rad/s, s  (gyro after ~16 Hz LPF)
NS.seed = 2;

%% --- Cases to run (same mission, same gusts, same noise) ---
CASES = struct( ...
  'name',  {'Ideal',  'Wind + gusts', 'Wind + gusts + noise'}, ...
  'wind',  {false,    true,           true}, ...
  'noise', {false,    false,          true});

%% --- Simulation ---
SIM.dt    = 0.004;   % s, fixed-step RK4 (250 Hz, like a flight controller)
SIM.t_end = Inf;     % s, Inf = whole mission; shorten for quick tests
SIM.v_cap = 1.0;     % m/s, velocity-command cap (mission is 0.5 m/s; the
                     % extra room lets the position loop correct drift)
SIM.plot_case = numel(CASES);   % which case gets the detailed figures
SIM.save_png  = true;

%% =======================================================================

%% Mission profile from the planner, sampled at the controller rate
M.stage_names = {'takeoff', 'transit', 'search', 'return', 'landing'};
[~, ~, ~, ~, mission] = gn_build_mission(cfg);
clear mission_sequencer
dt_m = cfg.ctrlTs;
K = ceil((sum(mission.T) + 10)/dt_m);
M.t = (0:K-1).'*dt_m;
M.pos = zeros(K, 3);  M.vel = M.pos;  M.acc = M.pos;  M.stage = zeros(K, 1);
p_meas = mission.home_ned_m;
for k = 1:K
    [M.stage(k), t_in] = mission_sequencer(mission, M.t(k), true, p_meas);
    [p_meas, v_k, a_k] = mission_eval(mission, M.stage(k), t_in);
    M.pos(k,:) = p_meas.';  M.vel(k,:) = v_k.';  M.acc(k,:) = a_k.';
    if M.stage(k) == 6, break; end            % 6 = mission complete
end
M.t = M.t(1:k);  M.pos = M.pos(1:k,:);  M.vel = M.vel(1:k,:);  M.acc = M.acc(1:k,:);
M.stage = M.stage(1:k);

%% Power curve: P_motor = P_hover*(f/f_hover)^n through the sheet's hover and
%% 1.5x-hover points (columns: AUW, hover gf/rotor, 1.5x gf/rotor, hover W x4,
%% peak W x4, battery g)
pw  = readcell(PW.xlsx, 'Sheet', 'Sheet3');
tab = cell2mat(pw(6:end, 1:6));
row = tab(abs(tab(:,1) - PW.AUW) < 1e-6, :);
PW.f_hover   = row(2)*9.80665e-3;             % N per rotor
PW.P_hover   = row(4)/4;                      % W per motor
PW.n         = log(row(5)/row(4))/log(row(3)/row(2));
PW.avionics  = pw{2, 2};                      % W
PW.batt_g    = row(6);
PW.E_batt_Wh = row(6)/1000*pw{1, 10}*pw{1, 6};   % g -> kg * Wh/kg * usable fraction

t_end = min(SIM.t_end, M.t(end) + 2);           % +2 s to settle after landing
t  = (0:SIM.dt:t_end).';
Nt = numel(t);

%% Controller reference struct: your GB gains in 'velocity' mode
ref0 = GB;
ref0.x_mode = 'velocity';  ref0.y_mode = 'velocity';  ref0.z_mode = 'velocity';
ref0.vx = 0;  ref0.vy = 0;  ref0.vz = 0;
ref0.x  = 0;  ref0.y  = 0;  ref0.z  = 0;
ref0.psi = 0;
ref0.u_max = SIM.v_cap;  ref0.v_max = SIM.v_cap;  ref0.w_max = SIM.v_cap;

%% Disturbance time series (shared by all cases that use them)
W.V = W.speed;
[gust, ginfo] = DrydenGusts(t, W);
chi = deg2rad(W.to_deg);
wind_mean = W.speed*[cos(chi), sin(chi), 0];
wind_all  = wind_mean + gust;

randn('seed', NS.seed);  try, rng(NS.seed); catch, end
sig12 = [NS.pos*[1 1 1], NS.att*[1 1 1], NS.vel*[1 1 1], NS.rate*[1 1 1]];
tau12 = [NS.tau_pos*[1 1 1], NS.tau_att*[1 1 1], NS.tau_vel*[1 1 1], NS.tau_rate*[1 1 1]];
a12   = exp(-SIM.dt./tau12);                    % AR(1): n_k = a n_{k-1} + sqrt(1-a^2) sigma e_k
e     = randn(Nt, 12);
noise_all = zeros(Nt, 12);
noise_all(1,:) = sig12.*e(1,:);
for k = 2:Nt
    noise_all(k,:) = a12.*noise_all(k-1,:) + sqrt(1 - a12.^2).*sig12.*e(k,:);
end

fprintf('Dryden gusts: sigma_u = %.2f, sigma_w = %.2f m/s, L_u = %.1f m, L_w = %.1f m\n', ...
        ginfo.sigma_u, ginfo.sigma_w, ginfo.L_u, ginfo.L_w);
fprintf('Drag nu = %.4g N/(m/s)^2, motor tau = %.3f s, f_max = %.1f N\n\n', ...
        P.nu, P.tau_m, P.f_max);

%% Run every case
stg = interp1(M.t, M.stage, min(t, M.t(end)), 'previous');
stg(t > M.t(end)) = 6;
hover_f = P.m*P.g/4;
results = struct([]);
for c = 1:numel(CASES)
    C = CASES(c);
    fprintf('Running case %d/%d: %s ... ', c, numel(CASES), C.name);  tic;

    wind  = zeros(Nt, 3);  if C.wind,  wind  = wind_all;  end
    noise = zeros(Nt, 12); if C.noise, noise = noise_all; end

    % Start in hover at the first mission point, motors at hover thrust
    x = zeros(27, 1);
    x(1:3)   = M.pos(1,:).';
    x(24:27) = hover_f;

    X  = zeros(Nt, 27);
    Fa = zeros(Nt, 4);  Fcmd = zeros(Nt, 4);
    Pref = zeros(Nt, 3);  Gc_h = zeros(Nt, 3);
    for k = 1:Nt
        wk = wind(k,:).';  nk = noise(k,:).';
        [k1, info] = RobustClosedLoopEOM(t(k), x, P, ref0, M, wk, nk);
        X(k,:) = x.';
        Fa(k,:) = info.f_act.';  Fcmd(k,:) = info.f_cmd.';
        Pref(k,:) = info.p_ref.';  Gc_h(k,:) = info.Gc.';
        if k == Nt, break; end
        h  = SIM.dt;
        k2 = RobustClosedLoopEOM(t(k) + h/2, x + h/2*k1, P, ref0, M, wk, nk);
        k3 = RobustClosedLoopEOM(t(k) + h/2, x + h/2*k2, P, ref0, M, wk, nk);
        k4 = RobustClosedLoopEOM(t(k) + h,   x + h*k3,   P, ref0, M, wk, nk);
        x  = x + h/6*(k1 + 2*k2 + 2*k3 + k4);
        if any(~isfinite(x))
            error('Case "%s" diverged at t = %.2f s', C.name, t(k));
        end
    end
    fprintf('%.1f s\n', toc);

    % --- Metrics ---
    dF   = Fa - mean(Fa, 2);
    perr = X(:,1:3) - Pref;
    R = struct();
    R.name   = C.name;
    R.t = t;  R.X = X;  R.Fa = Fa;  R.Fcmd = Fcmd;  R.Pref = Pref;  R.Gc = Gc_h;
    R.wind = wind;
    R.f_peak     = max(Fa(:));
    R.f_min      = min(Fa(:));
    R.pct_max    = 100*R.f_peak/P.f_max;
    R.sat_pct    = 100*mean(any(Fcmd > P.f_max | Fcmd < P.f_min, 2));
    R.dF_max     = max(abs(dF(:)));
    R.dF_rms     = sqrt(mean(dF(:).^2));
    R.tilt_max   = rad2deg(max(acos(cos(X(:,4)).*cos(X(:,5)))));
    R.herr_max   = max(sqrt(sum(perr(:,1:2).^2, 2)));
    R.herr_rms   = sqrt(mean(sum(perr(:,1:2).^2, 2)));
    R.zerr_max   = max(abs(perr(:,3)));
    R.rpm_peak   = sqrt(R.f_peak/P.kT)*60/(2*pi);
    R.Pm         = motor_power(Fa, PW.f_hover, PW.P_hover, PW.n);   % W per motor
    R.P_W        = sum(R.Pm, 2) + PW.avionics;                      % W total
    R.E_Wh       = cumtrapz(t, R.P_W)/3600;
    R.E_stage_Wh = zeros(1, 6);                                     % 6 = settle
    for s = 1:6
        in = stg == s;
        R.E_stage_Wh(s) = trapz(t(in), R.P_W(in))/3600;
    end
    if isempty(results), results = R; else, results(c) = R; end
end

%% Summary table
fprintf('\nHover thrust per motor mg/4 = %.3f N  (%.1f%% of f_max)\n\n', hover_f, 100*hover_f/P.f_max);
fprintf('%-24s %8s %8s %8s %8s %9s %9s %8s %9s %8s\n', 'Case', 'f_peak', 'f_min', '%f_max', ...
        'sat %', 'max|df|', 'rms df', 'tilt', 'max xy e', 'max z e');
fprintf('%-24s %8s %8s %8s %8s %9s %9s %8s %9s %8s\n', '', '[N]', '[N]', '', '[time]', '[N]', '[N]', ...
        '[deg]', '[m]', '[m]');
for c = 1:numel(results)
    R = results(c);
    fprintf('%-24s %8.3f %8.3f %8.1f %8.2f %9.4f %9.4f %8.2f %9.3f %8.3f\n', R.name, R.f_peak, ...
            R.f_min, R.pct_max, R.sat_pct, R.dF_max, R.dF_rms, R.tilt_max, R.herr_max, R.zerr_max);
end
fprintf('\n(df = f_i minus the mean of the four motors at that instant)\n');

%% Power summary
fprintf('\nAUW %.1f kg | hover %.0f gf/rotor -> %.0f W (4 motors) + %.0f W avionics | P ~ f^%.3f\n', ...
        PW.AUW, row(2), row(4), PW.avionics, PW.n);
fprintf('Usable battery %.0f Wh (%.0f g, %.0f Wh/kg, %.0f%% usable) | mission %.1f s\n\n', ...
        PW.E_batt_Wh, PW.batt_g, pw{1, 10}, 100*pw{1, 6}, M.t(end));
fprintf('%-24s %8s %8s %8s %8s %9s\n', 'Case', 'avg', 'peak', 'energy', 'battery', 'endurance');
fprintf('%-24s %8s %8s %8s %8s %9s\n', '', '[W]', '[W]', '[Wh]', '[%]', '[min]');
for c = 1:numel(results)
    R = results(c);
    avgW = R.E_Wh(end)*3600/t(end);
    fprintf('%-24s %8.1f %8.1f %8.2f %8.1f %9.1f\n', R.name, avgW, max(R.P_W), R.E_Wh(end), ...
            100*R.E_Wh(end)/PW.E_batt_Wh, 60*PW.E_batt_Wh/avgW);
end
fprintf('\nEnergy by stage [Wh] (%s):\n', results(SIM.plot_case).name);
stage_lbl = [M.stage_names, {'settle'}];
for s = 1:6
    fprintf('  %-8s %6.2f\n', stage_lbl{s}, results(SIM.plot_case).E_stage_Wh(s));
end

%% Figures + CSV
fig_dir = fullfile(this_dir, 'figures');
if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end
PlotMotorfRobust(results, SIM.plot_case, P, M, fig_dir, SIM.save_png);
PlotMissionPower(results, SIM.plot_case, M, stg, PW, fig_dir, SIM.save_png);

Rp = results(SIM.plot_case);
csv_name = fullfile(fig_dir, 'motor_forces_robust.csv');
fid = fopen(csv_name, 'w');
fprintf(fid, ['t_s,f1_N,f2_N,f3_N,f4_N,f1_cmd_N,f2_cmd_N,f3_cmd_N,f4_cmd_N,wind_N_mps,wind_E_mps,wind_D_mps,' ...
              'P1_W,P2_W,P3_W,P4_W,P_total_W,E_Wh\n']);
fprintf(fid, ['%.3f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,' ...
              '%.2f,%.2f,%.2f,%.2f,%.2f,%.4f\n'], ...
        [Rp.t, Rp.Fa, Rp.Fcmd, Rp.wind, Rp.Pm, Rp.P_W, Rp.E_Wh].');
fclose(fid);
fprintf('Wrote %s\n', csv_name);
end
