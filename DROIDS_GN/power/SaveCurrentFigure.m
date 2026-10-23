function SaveCurrentFigure(fig_dir, filename)
% SaveCurrentFigure - Saves the current figure (gcf) as a PNG into
% fig_dir, creating the folder if it doesn't exist yet. Call this right
% after each PlotXxx(...) call, while that plot is still the current
% figure.
%
% Inputs:
%   fig_dir  - folder to save into (created if missing)
%   filename - output filename, e.g. 'bodytilt_all_states.png'

if ~exist(fig_dir, 'dir')
    mkdir(fig_dir);
end

fig = gcf;

% Force a light theme/white background regardless of the system's dark
% mode -- MATLAB (R2023b+) applies the OS appearance to new figures by
% default, which saveas then bakes into the PNG as a black background.
% theme() may not exist on older MATLAB/Octave, so fall back to setting
% colors directly either way.
try
    theme(fig, 'light');
catch
    % theme() unavailable -- the explicit color-setting below still runs
end
set(fig, 'Color', 'w');
ax_list = findall(fig, 'Type', 'axes');
for k = 1:numel(ax_list)
    set(ax_list(k), 'Color', 'w', 'XColor', 'k', 'YColor', 'k');
end

saveas(fig, fullfile(fig_dir, filename));

end
