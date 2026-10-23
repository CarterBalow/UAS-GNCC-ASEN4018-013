function PlotMotorfRobust(results, ic, P, M, fig_dir, save_png)
% PlotMotorfRobust - Figures for main_motorf_robust:
%   1) Motor forces for case ic: f_i, f_i - mean, tilt, wind
%   2) Mission tracking for case ic: position vs. mission, horizontal/vertical error
%   3) Case comparison: differential thrust and per-case peak/min thrust
%
% Inputs:
%   results  - struct array from main_motorf_robust
%   ic       - index of the case to plot in detail
%   P        - params (f_max, m, g)
%   M        - mission struct (unused; kept for future stage shading)
%   fig_dir  - folder for PNGs
%   save_png - true to save each figure with SaveCurrentFigure

R = results(ic);
t = R.t;
hover_f = P.m*P.g/4;
tag = lower(regexprep(R.name, '[^A-Za-z0-9]+', '_'));

%% 1) Motor forces
figure('Name', ['Motor forces - ' R.name], 'NumberTitle', 'off');
ax(1) = subplot(4,1,1);
plot(t, R.Fa); hold on; plot(t([1 end]), hover_f*[1 1], 'k:');
ylabel('f_i [N]'); legend('f_1','f_2','f_3','f_4','hover','Location','northeast','Orientation','horizontal');
title(sprintf('Per-motor thrust - %s  (peak %.2f N = %.0f%% of f_{max})', ...
      R.name, R.f_peak, R.pct_max)); grid on;
ax(2) = subplot(4,1,2);
plot(t, R.Fa - mean(R.Fa, 2));
ylabel('f_i - mean [N]'); legend('f_1','f_2','f_3','f_4','Location','northeast','Orientation','horizontal');
title('Differential thrust'); grid on;
ax(3) = subplot(4,1,3);
plot(t, rad2deg(R.X(:,4:5)));
ylabel('[deg]'); legend('\phi','\theta','Location','northeast','Orientation','horizontal'); grid on;
title('Attitude');
ax(4) = subplot(4,1,4);
plot(t, R.wind);
ylabel('[m/s]'); xlabel('t [s]'); legend('w_N','w_E','w_D','Location','northeast','Orientation','horizontal'); grid on;
title('Wind (mean + gust)');
linkaxes(ax, 'x');  xlim(ax(1), t([1 end]));
if save_png, SaveCurrentFigure(fig_dir, ['motor_forces_' tag '.png']); end

%% 2) Mission tracking
figure('Name', ['Mission tracking - ' R.name], 'NumberTitle', 'off');
subplot(2,2,[1 3]);
plot(R.Pref(:,2), R.Pref(:,1), 'k--'); hold on;
plot(R.X(:,2), R.X(:,1), 'b');
axis equal; grid on; xlabel('y_E [m]'); ylabel('x_N [m]');
legend('mission', 'flown', 'Location', 'best'); title('Ground track');
perr = R.X(:,1:3) - R.Pref;
bx(1) = subplot(2,2,2);
plot(t, sqrt(sum(perr(:,1:2).^2, 2))); grid on;
ylabel('|e_{xy}| [m]'); title('Horizontal tracking error');
bx(2) = subplot(2,2,4);
plot(t, -R.X(:,3), 'b', t, -R.Pref(:,3), 'k--'); grid on;
ylabel('altitude [m]'); xlabel('t [s]'); legend('flown','mission','Location','best');
title('Altitude');
linkaxes(bx, 'x');  xlim(bx(1), t([1 end]));
if save_png, SaveCurrentFigure(fig_dir, ['tracking_' tag '.png']); end

%% 3) Case comparison
figure('Name', 'Motor forces - case comparison', 'NumberTitle', 'off');
nc = numel(results);
subplot(2,1,1); hold on;
for c = 1:nc
    dF = results(c).Fa - mean(results(c).Fa, 2);
    plot(results(c).t, max(abs(dF), [], 2));
end
grid on; ylabel('max_i |f_i - mean| [N]'); xlabel('t [s]'); xlim(t([1 end]));
legend({results.name}, 'Location', 'best');
title('Largest motor-to-motor deviation, by case');
subplot(2,1,2);
vals = [[results.f_min]; [results.f_peak]].';
bar(vals); hold on;
plot([0.5 nc+0.5], hover_f*[1 1], 'k:');
set(gca, 'XTick', 1:nc, 'XTickLabel', {results.name});
ylabel('[N]'); legend('min f_i', 'peak f_i', 'hover', 'Location', 'best'); grid on;
title(sprintf('Thrust envelope per case (f_{max} = %.1f N)', P.f_max));
if save_png, SaveCurrentFigure(fig_dir, 'case_comparison.png'); end
end
