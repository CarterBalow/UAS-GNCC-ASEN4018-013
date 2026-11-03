function PlotMissionPower(results, ic, M, stg, PW, fig_dir, save_png)
% PlotMissionPower - Total power (all cases), per-motor power (case ic) and
% cumulative energy over the mission, with the mission stages shaded.

t = results(1).t;
t_stage = [arrayfun(@(s) t(find(stg == s, 1)), 1:5), M.t(end)];
stage_colors = [0.5 0.5 0.5; 1 0 0; 0 0 1; 1 0 1; 0.2 0.2 0.2];
Rp = results(ic);

figure('Name', 'Power Draw', 'NumberTitle', 'off');
for sp = 1:3
    subplot(3, 1, sp); hold on; grid on;
    xlim([0 t(end)]);
    for j = 1:5
        xregion(t_stage(j), t_stage(j+1), 'FaceColor', stage_colors(j,:), 'FaceAlpha', 0.08, ...
                'DisplayName', M.stage_names{j});
    end
    if sp == 1
        for c = numel(results):-1:1
            plot(t, results(c).P_W, 'LineWidth', 1.1, 'DisplayName', results(c).name);
        end
        yline(4*PW.P_hover + PW.avionics, 'k:', 'DisplayName', 'sheet hover');
        ylabel('Total power [W]');
        title(sprintf('Electrical power, %.1f kg AUW (P \\propto f^{%.2f}, + %.0f W avionics)', ...
              PW.AUW, PW.n, PW.avionics));
        legend('Location', 'eastoutside');
    elseif sp == 2
        plot(t, Rp.Pm, 'LineWidth', 1);
        yline(PW.P_hover, 'k:');
        ylabel('Per motor [W]');
        legend([repmat({''}, 1, 5), {'P_1', 'P_2', 'P_3', 'P_4', 'hover'}], 'Location', 'eastoutside');
        title(['Per-motor power - ' Rp.name]);
    else
        % Hover dominates, so every case's E(t) is a near-identical straight
        % line; plot the extra energy each case uses over Ideal instead.
        E0 = results(1).E_Wh;
        for c = numel(results):-1:1
            plot(t, results(c).E_Wh - E0, 'LineWidth', 1.1, 'DisplayName', ...
                 sprintf('%s (%.2f Wh)', results(c).name, results(c).E_Wh(end)));
        end
        ylabel(['\DeltaE vs ' results(1).name ' [Wh]']); xlabel('Time [s]');
        title(sprintf('Extra energy over %s (%.2f Wh total, usable battery %.0f Wh)', ...
              results(1).name, E0(end), PW.E_batt_Wh));
        legend('Location', 'eastoutside');
    end
end
if save_png, SaveCurrentFigure(fig_dir, 'power_draw.png'); end
end
