%% UAV simulation runner and 3D mission plot
% Save this file in UAV_GNC_Project or its scripts folder and click Run.
% Keep the project root as MATLAB's Current Folder for gn_init.
% No gn_collectLogs helper is required. Local functions are included below.
% The model's phase, targets, enable wiring, and initial position stay as set
% in Simulink. This script runs the current harness or complete mission.

%% Settings you can change
gnRunSeconds = 60;                  % Simulated seconds: 30, 60, 300, 600, etc.
gnModelName = 'gncc_uav_toolbox';
gnInitialize = true;                % Run gn_init before each simulation.
gnShowSearchPath = true;            % Overlay searchW when available.
gnShowHome = true;                  % Mark cfg.home when available.
gnGoalNED = [];                     % Optional marker, e.g. [2;1;-1.5].
gnSaveResults = false;              % Save a dated MAT file and PNG if true.

%% Locate the project files
% Add paths so the model and initialization can be found from either folder.
gnScriptFolder = fileparts(mfilename('fullpath'));
[~,gnFolderName] = fileparts(gnScriptFolder);
if strcmpi(gnFolderName,'scripts')
    gnProjectRoot = fileparts(gnScriptFolder);
else
    gnProjectRoot = gnScriptFolder;
end
addpath(gnProjectRoot);
for gnSubfolder = {'functions','scripts','models'}
    gnFolder = fullfile(gnProjectRoot,gnSubfolder{1});
    if isfolder(gnFolder)
        addpath(gnFolder);
    end
end

validateattributes(gnRunSeconds,{'numeric'}, ...
    {'scalar','real','finite','positive'});
if gnInitialize
    assert(exist('gn_init','file')~=0, ...
        'Cannot find gn_init.m. Save it in the project scripts folder.');
    gn_init;
end

%% Run a fresh simulation for the requested duration
% SimulationInput overrides apply to this run without saving model changes.
% Starting at zero and not loading a saved state gives a fresh test.
load_system(gnModelName);
assert(strcmp(get_param(gnModelName,'SimulationStatus'),'stopped'), ...
    'Stop the current simulation before running gn_main.');
gnSimInput = Simulink.SimulationInput(gnModelName);
gnSimInput = setModelParameter(gnSimInput, ...
    'StartTime','0', ...
    'StopTime',num2str(gnRunSeconds,17), ...
    'LoadInitialState','off', ...
    'ReturnWorkspaceOutputs','on', ...
    'SignalLogging','on', ...
    'SignalLoggingName','logsout');

fprintf('Running %s for %.1f simulated seconds...\n', ...
    gnModelName,gnRunSeconds);
gnTimer = tic;
out = sim(gnSimInput);
fprintf('Simulation finished in %.2f wall-clock seconds.\n',toc(gnTimer));

%% Read the actual aircraft position
% Prefer marked wire logs, then use a To Workspace timeseries if present.
% Select State's WorldPosition must be logged for this script to plot it.
[gnPositionSeries,gnPositionSource] = gnReadSeries(out, ...
    {'rawPositionLog','WorldPosition','pRaw'});
assert(~isempty(gnPositionSeries), ...
    ['Position not found. Log the WorldPosition wire with Custom name ' ...
     'rawPositionLog, or connect a To Workspace block named ' ...
     'rawPositionLog with Save format Timeseries, then rerun.']);
[t,P] = gnSeriesMatrix(gnPositionSeries,3);

% NED has positive Down, so negate Down to display altitude above the origin.
north = P(:,1);
east = P(:,2);
altitude = -P(:,3);
gnPlotPoints = [east,north,altitude];

%% Create the 3D mission plot
gnFigure = figure('Name','UAV 3D Mission','Color','w');
gnAxes = axes('Parent',gnFigure);
hold(gnAxes,'on');

if gnShowSearchPath && exist('searchW','var') && ~isempty(searchW)
    validateattributes(searchW,{'numeric'},{'2d','ncols',3,'finite'});
    gnPlanned = [searchW(:,2),searchW(:,1),-searchW(:,3)];
    plot3(gnAxes,gnPlanned(:,1),gnPlanned(:,2),gnPlanned(:,3), ...
        'k--','LineWidth',1,'DisplayName','Planned search path');
    gnPlotPoints = [gnPlotPoints;gnPlanned];
end

plot3(gnAxes,east,north,altitude,'b-','LineWidth',2, ...
    'DisplayName','Actual flight');
plot3(gnAxes,east(1),north(1),altitude(1),'go', ...
    'MarkerFaceColor','g','MarkerSize',8,'DisplayName','Start');
plot3(gnAxes,east(end),north(end),altitude(end),'ro', ...
    'MarkerFaceColor','r','MarkerSize',8,'DisplayName','End of run');

if gnShowHome && exist('cfg','var') && isstruct(cfg) && isfield(cfg,'home')
    gnHome = cfg.home(:);
    validateattributes(gnHome,{'numeric'},{'numel',3,'finite'});
    gnHomePlot = [gnHome(2),gnHome(1),-gnHome(3)];
    plot3(gnAxes,gnHomePlot(1),gnHomePlot(2),gnHomePlot(3),'ks', ...
        'MarkerFaceColor',[0.7 0.7 0.7],'MarkerSize',8, ...
        'DisplayName','Home');
    gnPlotPoints = [gnPlotPoints;gnHomePlot];
end

if ~isempty(gnGoalNED)
    validateattributes(gnGoalNED,{'numeric'},{'vector','numel',3,'finite'});
    gnGoalPlot = [gnGoalNED(2),gnGoalNED(1),-gnGoalNED(3)];
    plot3(gnAxes,gnGoalPlot(1),gnGoalPlot(2),gnGoalPlot(3),'kp', ...
        'MarkerFaceColor','y','MarkerSize',12,'DisplayName','Goal marker');
    gnPlotPoints = [gnPlotPoints;gnGoalPlot];
end

xlabel(gnAxes,'East (m)');
ylabel(gnAxes,'North (m)');
zlabel(gnAxes,'Altitude above NED origin (m)');
title(gnAxes,sprintf('UAV path through %.1f simulated seconds',t(end)));
grid(gnAxes,'on');
axis(gnAxes,'equal');
view(gnAxes,3);
legend(gnAxes,'show','Location','best');

% Fit the complete path and visible reference markers, including flat axes.
gnLow = min(gnPlotPoints,[],1);
gnHigh = max(gnPlotPoints,[],1);
gnMargin = max(0.15,0.1*(gnHigh-gnLow));
xlim(gnAxes,[gnLow(1)-gnMargin(1),gnHigh(1)+gnMargin(1)]);
ylim(gnAxes,[gnLow(2)-gnMargin(2),gnHigh(2)+gnMargin(2)]);
zlim(gnAxes,[min(0,gnLow(3))-gnMargin(3),gnHigh(3)+gnMargin(3)]);
rotate3d(gnFigure,'on');

% Force white backgrounds and black text.
set(gnFigure,'Color','w');

set(gnAxes, ...
    'Color','w', ...
    'XColor','k', ...
    'YColor','k', ...
    'ZColor','k', ...
    'GridColor',[0.65 0.65 0.65]);

gnAxes.Title.Color = 'k';

gnLegend = legend(gnAxes);
set(gnLegend, ...
    'Color','w', ...
    'TextColor','k', ...
    'EdgeColor',[0.7 0.7 0.7]);

%% Print numerical results without claiming mission completion
% A run ending at its stop time does not mean the search or mission is done.
fprintf('\nPosition source: %s\n',gnPositionSource);
fprintf('Logged interval: %.3f to %.3f s (%d samples).\n', ...
    t(1),t(end),numel(t));
fprintf('Final [North East Down]: [%.3f %.3f %.3f] m\n',P(end,:));
fprintf('Final altitude: %.3f m; peak altitude: %.3f m\n', ...
    altitude(end),max(altitude));

[gnVelocitySeries,~] = gnReadSeries(out, ...
    {'rawVelocityLog','WorldVelocity','vRaw'});
if ~isempty(gnVelocitySeries)
    [~,V] = gnSeriesMatrix(gnVelocitySeries,3);
    fprintf('Final speed: %.3f m/s; peak logged speed: %.3f m/s\n', ...
        norm(V(end,:)),max(vecnorm(V,2,2)));
end
if ~isempty(gnGoalNED)
    fprintf('Distance from goal marker: %.3f m\n', ...
        norm(P(end,:).'-gnGoalNED(:)));
end

%% Optional result files
if gnSaveResults
    gnResultsFolder = fullfile(gnProjectRoot,'results');
    if ~isfolder(gnResultsFolder), mkdir(gnResultsFolder); end
    gnResultBase = fullfile(gnResultsFolder, ...
        ['mission_' datestr(now,'yyyymmdd_HHMMSS_FFF')]);
    save([gnResultBase '.mat'],'out','t','P','gnRunSeconds','gnModelName');
    exportgraphics(gnFigure,[gnResultBase '.png'],'Resolution',200);
    fprintf('Saved MAT data and PNG plot in %s\n',gnResultsFolder);
end

%% Local helpers -- keep these at the bottom of this same file
function [series,source] = gnReadSeries(simOut,candidates)
% Find a named wire log or To Workspace output without external helpers.
series = [];
source = '';
outputNames = who(simOut);
if any(strcmp(outputNames,'logsout'))
    dataset = simOut.logsout;
    if isa(dataset,'Simulink.SimulationData.Dataset')
        names = getElementNames(dataset);
        for k = 1:numel(candidates)
            matches = find(strcmp(names,candidates{k}));
            assert(numel(matches)<=1, ...
                'Duplicate logging name %s. Assign unique names.',candidates{k});
            if ~isempty(matches)
                signal = getElement(dataset,matches(1));
                series = signal.Values;
                source = ['logsout: ' candidates{k}];
                return
            end
        end
    end
end
for k = 1:numel(candidates)
    if any(strcmp(outputNames,candidates{k}))
        series = simOut.(candidates{k});
        source = ['To Workspace: ' candidates{k}];
        return
    end
end
end

function [time,data] = gnSeriesMatrix(series,nComponents)
% Handle both N-by-3 vector data and 3-by-1-by-N matrix data.
assert(isa(series,'timeseries'), ...
    'Use Timeseries for To Workspace or timeseries Dataset signal format.');
time = series.Time(:);
n = numel(time);
assert(n>=2,'Fewer than two logged samples. Disable logging limits and rerun.');
assert(numel(series.Data)==n*nComponents, ...
    'Expected %d components per sample. Log the entire vector.',nComponents);
if series.IsTimeFirst
    data = reshape(series.Data,n,nComponents);
else
    data = reshape(series.Data,nComponents,n).';
end
assert(all(isfinite(time)) && all(isfinite(data(:))), ...
    'Logged data contains NaN or Inf. Check the model before plotting.');
assert(all(diff(time)>=0),'Logged timestamps are not ordered.');
end
