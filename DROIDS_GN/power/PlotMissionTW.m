function PlotMissionTW(results, ic, M, stg, P, fig_dir, save_png)
% PlotMissionTW - Thrust-to-weight over the mission, with the mission stages
% shaded (same layout as PlotMissionPower):
%   1) total T/W = sum(f_i)/(m g), all cases, with the available T/W
%   2) per-motor T/W = f_i/(m g/4), case ic (f_max limit given in the title)
%   3) thrust headroom on the most-loaded motor, 1 - max_i(f_i)/f_max, all cases

t = results(1).t;
t_stage = [arrayfun(@(s) t(find(stg == s, 1)), 1:5), M.t(end)];
stage_colors = [0.5 0.5 0.5; 1 0 0; 0 0 1; 1 0 1; 0.2 0.2 0.2];
Rp = results(ic);
W  = P.m*P.g;                 % N, weight
TW_max = 4*P.f_max/W;         % available T/W at full throttle

figure('Name', 'Thrust to Weight', 'NumberTitle', 'off');
for sp = 1:3
    subplot(3, 1, sp); hold on; grid on;
    xlim([0 t(end)]);
    for j = 1:5
        xregion(t_stage(j), t_stage(j+1), 'FaceColor', stage_colors(j,:), 'FaceAlpha', 0.08, ...
                'DisplayName', M.stage_names{j});
    end
    if sp == 1
        for c = numel(results):-1:1
            TW = sum(results(c).Fa, 2)/W;
            plot(t, TW, 'LineWidth', 1.1, 'DisplayName', ...
                 sprintf('%s (peak %.2f)', results(c).name, max(TW)));
        end
        yline(1, 'k:', 'DisplayName', 'hover (T/W = 1)');
        yline(TW_max, 'r--', 'DisplayName', sprintf('available (%.2f)', TW_max));
        ylabel('Total T/W [-]');
        title(sprintf('Thrust-to-weight, %.1f kg AUW (max available %.2f)', P.m, TW_max));
        legend('Location', 'eastoutside');
    elseif sp == 2
        plot(t, Rp.Fa/(W/4), 'LineWidth', 1);
        yline(1, 'k:');
        ylabel('Per motor T/W [-]');
        legend([repmat({''}, 1, 5), {'f_1', 'f_2', 'f_3', 'f_4', 'hover'}], 'Location', 'eastoutside');
        title(sprintf('Per-motor thrust / (mg/4) - %s (f_{max} = %.2f)', Rp.name, TW_max));
    else
        for c = numel(results):-1:1
            hr = 100*(1 - max(results(c).Fa, [], 2)/P.f_max);
            plot(t, hr, 'LineWidth', 1.1, 'DisplayName', ...
                 sprintf('%s (min %.0f%%)', results(c).name, min(hr)));
        end
        yline(100*(1 - (W/4)/P.f_max), 'k:', 'DisplayName', 'hover');
        ylabel('Headroom [% f_{max}]'); xlabel('Time [s]');
        title('Thrust headroom on the most-loaded motor, 1 - max_i f_i / f_{max}');
        legend('Location', 'eastoutside');
    end
end
if save_png, SaveCurrentFigure(fig_dir, 'thrust_to_weight.png'); end
end
