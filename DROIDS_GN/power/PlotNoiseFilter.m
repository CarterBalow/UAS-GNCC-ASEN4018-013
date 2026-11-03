function PlotNoiseFilter(R, fig_dir, save_png)
% PlotNoiseFilter - Sensor noise vs. what the filters hand the controller,
% for one ESKF case. Left column: raw sensor error (measurement - truth).
% Right column: estimate error after the ESKF (and gyro low-pass). Each row
% shares its y-limits, so the reduction reads directly.
%
% Rows: north position (GPS), down position (baro + GPS), down velocity (GPS),
%       roll rate (gyro -> bias-corrected + low-pass).

S = R.sens;
if isempty(S), return; end
t = R.t;  E = R.est_err;
c_raw = [0.85 0.33 0.10];  c_flt = [0 0.45 0.74];  c_alt = [0.47 0.67 0.19];
rmsf = @(x) sqrt(mean(x(~isnan(x)).^2));
sel  = t >= 5;                              % skip filter convergence in the RMS

rows = {
  % label, units, scale, raw {t, y, name}, raw2 (optional), filtered
  'North position', 'm',     1,        {S.t_gps, S.gps_p(:,1), 'GPS'},        {},                              E(:,1)
  'Down position',  'm',     1,        {S.t_gps, S.gps_p(:,3), 'GPS'},        {S.t_baro, S.baro_D, 'baro'},    E(:,3)
  'Down velocity',  'm/s',   1,        {S.t_gps, S.gps_v(:,3), 'GPS'},        {},                              S.vel_N(:,3)
  'Roll rate',      'deg/s', 180/pi,   {t, S.gyro(:,1), 'gyro'},              {},                              E(:,10)
};
nr = size(rows, 1);

figure('Name', ['Noise vs Filter - ' upper(S.mode)], 'NumberTitle', 'off');
for i = 1:nr
    [lbl, un, sc, raw, raw2, flt] = rows{i,:};
    ax1 = subplot(nr, 2, 2*i - 1); hold on; grid on;
    if ~isempty(raw2)
        plot(raw2{1}, sc*raw2{2}, '.', 'Color', c_alt, 'MarkerSize', 3, 'DisplayName', ...
             sprintf('%s (rms %.3g)', raw2{3}, rmsf(sc*raw2{2})));
    end
    mk = '.';  if numel(raw{1}) > 5000, mk = '-'; end
    plot(raw{1}, sc*raw{2}, mk, 'Color', c_raw, 'MarkerSize', 5, 'DisplayName', ...
         sprintf('%s (rms %.3g)', raw{3}, rmsf(sc*raw{2})));
    ylabel(sprintf('%s [%s]', lbl, un));
    legend('Location', 'northeast');
    if i == 1, title('Noise only: raw sensor error'); end
    if i == nr, xlabel('Time [s]'); end

    ax2 = subplot(nr, 2, 2*i); hold on; grid on;
    if i == nr
        nm = sprintf('bias-corrected + %g Hz LPF', S.gyro_lpf_hz);
    else
        nm = 'ESKF estimate';
    end
    plot(t, sc*flt, '-', 'Color', c_flt, 'LineWidth', 1, 'DisplayName', ...
         sprintf('%s (rms %.3g)', nm, rmsf(sc*flt(sel))));
    legend('Location', 'northeast');
    if i == 1, title(['With filters: error sent to the controller (' upper(S.mode) ' GPS)']); end
    if i == nr, xlabel('Time [s]'); end

    linkaxes([ax1 ax2], 'xy');
    xlim(ax1, [t(1) t(end)]);
end
sgtitle(sprintf('Sensor noise vs. filtered estimate - %s', R.name));
if save_png, SaveCurrentFigure(fig_dir, sprintf('noise_filter_%s.png', S.mode)); end
end
