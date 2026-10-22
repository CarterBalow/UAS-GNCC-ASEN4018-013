%% UAV simulation runner and 3D mission plot
% Save this file in UAV_GNC_Project or its scripts folder and click Run.
% Keep the project root as MATLAB's Current Folder for gn_init.
% No gn_collectLogs helper is required. Local functions are included below.
% Click MATLAB Editor Run (F5) to initialize, animate, simulate, and plot.
% Requires the completed Step 11 model. Navigation means estimated state,
% not a planned trajectory. Euler inputs are [yaw pitch roll] in radians.
% Added logging/animation branches are display-only, in memory; save the
% model yourself if you want those branches retained in the SLX file.

%% Settings you can change
gnRunSeconds = [];                  % [] uses cfg.stopTime for the full mission.
gnLiveAnimation = true;             % Uses the native UAV Animation block.
gnPacing = true;                    % Slow execution enough to watch animation.
gnPacingRate = 5;                   % Simulated seconds per real second (1 = real time).
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

if gnInitialize
    assert(exist('gn_init','file')~=0, ...
        'Cannot find gn_init.m. Save it in the project scripts folder.');
    gn_init;
end

if isempty(gnRunSeconds)
    assert(exist('cfg','var') && isfield(cfg,'stopTime'), ...
        'gn_init must define cfg.stopTime for the full mission.');
    gnRunSeconds = cfg.stopTime;
end
validateattributes(gnRunSeconds,{'numeric'}, ...
    {'scalar','real','finite','positive'});
validateattributes(gnPacingRate,{'numeric'}, ...
    {'scalar','real','finite','positive'});

%% Run a fresh simulation for the requested duration
% SimulationInput overrides apply to this run without saving model changes.
% Starting at zero and not loading a saved state gives a fresh test.
load_system(gnModelName);
assert(strcmp(get_param(gnModelName,'SimulationStatus'),'stopped'), ...
    'Stop the current simulation before running gn_main.');
% Name the existing subsystem output wires for reliable data retrieval.
% The output block aliases also support the native plant signal names.
gnTapOutput(gnModelName,'Airborne Plant',{'pRaw','WorldPosition'},'rawPositionLog');
gnTapOutput(gnModelName,'Airborne Plant',{'vRaw','WorldVelocity'},'rawVelocityLog');
gnTapOutput(gnModelName,'Airborne Plant',{'eulerRaw','EulerZYX'},'rawEulerLog');
gnTapOutput(gnModelName,'Navigation',{'pNav'},'navPositionLog');
gnTapOutput(gnModelName,'Navigation',{'vNav'},'navVelocityLog');
gnTapOutput(gnModelName,'Navigation',{'eulerNav'},'navEulerLog');
gnTapOutput(gnModelName,'Mission Manager',{'phase'},'phaseLog');
gnTapOutput(gnModelName,'Mission Manager',{'pGoal'},'goalPositionLog');
if gnLiveAnimation
    gnPrepareAnimation(gnModelName);
end

% Normal mode supports the native animation. Pacing changes viewing speed,
% not simulated vehicle speed, solver step size, or controller behavior.
gnSimInput = Simulink.SimulationInput(gnModelName);
gnSimInput = setModelParameter(gnSimInput, ...
    'SimulationMode','normal', ...
    'StartTime','0', ...
    'StopTime',num2str(gnRunSeconds,17), ...
    'LoadInitialState','off', ...
    'ReturnWorkspaceOutputs','on', ...
    'SignalLogging','on', ...
    'SignalLoggingName','logsout');

if gnPacing && gnLiveAnimation
    gnSimInput = setModelParameter(gnSimInput, ...
        'EnablePacing','on','PacingRate',num2str(gnPacingRate,17));
else
    gnSimInput = setModelParameter(gnSimInput,'EnablePacing','off');
end

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

if gnShowHome && ((exist('home','var') && numel(home)==3) || ...
        (exist('cfg','var') && isstruct(cfg) && isfield(cfg,'home')))
    if exist('home','var') && numel(home)==3
        gnHome = home(:);
    else
        gnHome = cfg.home(:);
    end
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

%% Actual aircraft state versus navigation estimates
% Use each log's own timestamps: no assumption of equal sample rates.
% A perfect Navigation subsystem should overlap actual data exactly.
gnComparisonFigure = figure('Name','Actual vs Navigation State','Color','w');
gnTiles = tiledlayout(gnComparisonFigure,3,3,'TileSpacing','compact');
title(gnTiles,'Actual aircraft state vs navigation estimate');
gnPairs = { ...
    {'rawPositionLog','WorldPosition','pRaw'}, {'navPositionLog','pNav'}; ...
    {'rawVelocityLog','WorldVelocity','vRaw'}, {'navVelocityLog','vNav'}; ...
    {'rawEulerLog','rawAttitudeLog','EulerZYX','eulerRaw'}, ...
    {'navEulerLog','navAttitudeLog','eulerNav'}};
gnLabels = {{'North (m)','East (m)','Down (m)'}, ...
    {'North velocity (m/s)','East velocity (m/s)','Down velocity (m/s)'}, ...
    {'Roll (deg)','Pitch (deg)','Yaw (deg)'}};
for gnRow = 1:3
    [gnActual,~] = gnReadSeries(out,gnPairs{gnRow,1});
    [gnNav,~] = gnReadSeries(out,gnPairs{gnRow,2});
    gnActualTime = []; gnNavTime = []; gnActualData = []; gnNavData = [];
    if ~isempty(gnActual)
        [gnActualTime,gnActualData] = gnSeriesMatrix(gnActual,3);
    end
    if ~isempty(gnNav)
        [gnNavTime,gnNavData] = gnSeriesMatrix(gnNav,3);
    end
    if gnRow == 3
        % Plant and navigation EulerZYX: yaw, pitch, roll -> roll, pitch, yaw.
        if ~isempty(gnActualData), gnActualData = rad2deg(gnActualData(:,[3 2 1])); end
        if ~isempty(gnNavData), gnNavData = rad2deg(gnNavData(:,[3 2 1])); end
    end
    for gnCol = 1:3
        gnAx = nexttile(gnTiles,(gnRow-1)*3+gnCol);
        hold(gnAx,'on'); grid(gnAx,'on');
        if ~isempty(gnActualData)
            plot(gnAx,gnActualTime,gnActualData(:,gnCol),'b-', ...
                'LineWidth',1.5,'DisplayName','Actual (raw plant)');
        end
        if ~isempty(gnNavData)
            plot(gnAx,gnNavTime,gnNavData(:,gnCol),'r--', ...
                'LineWidth',1.2,'DisplayName','Navigation estimate');
        end
        ylabel(gnAx,gnLabels{gnRow}{gnCol}); xlabel(gnAx,'Time (s)');
        set(gnAx,'Color','w','XColor','k','YColor','k');
        if ~isempty(gnActualData) || ~isempty(gnNavData)
            gnLeg = legend(gnAx,'show','Location','best');
            set(gnLeg,'Color','w','TextColor','k');
        end
        if isempty(gnActualData) || isempty(gnNavData)
            text(gnAx,0.03,0.94,'Missing log: see Command Window', ...
                'Units','normalized','VerticalAlignment','top','Color',[0.65 0 0]);
        end
    end
    if isempty(gnActualData) || isempty(gnNavData)
        warning('gn_main:MissingComparison', ...
            'Missing comparison logs. Actual aliases: %s. Navigation aliases: %s.', ...
            strjoin(gnPairs{gnRow,1},', '),strjoin(gnPairs{gnRow,2},', '));
    end
end

%% Mission phase history and completion check
[gnPhase,~] = gnReadSeries(out,{'phaseLog','phase'});
if ~isempty(gnPhase)
    [gnPhaseTime,gnPhaseValue] = gnSeriesMatrix(gnPhase,1);
    gnPhaseFigure = figure('Name','Mission Phase History','Color','w');
    gnPhaseAxes = axes('Parent',gnPhaseFigure);
    stairs(gnPhaseAxes,gnPhaseTime,gnPhaseValue,'LineWidth',1.5);
    grid(gnPhaseAxes,'on'); xlabel(gnPhaseAxes,'Time (s)');
    yticks(gnPhaseAxes,0:6);
    yticklabels(gnPhaseAxes,{'PRECHECK','TAKEOFF','TRANSIT','SEARCH', ...
        'RETURN','LAND','COMPLETE'});
    ylim(gnPhaseAxes,[-0.25 6.25]);
    set(gnPhaseAxes,'Color','w','XColor','k','YColor','k');
    gnChanged = [true;diff(gnPhaseValue)~=0];
    disp(table(gnPhaseTime(gnChanged),gnPhaseValue(gnChanged), ...
        'VariableNames',{'Time_s','Phase'}));
    if isequal(gnPhaseValue(gnChanged).',0:6)
        fprintf('All seven mission phases occurred in order.\n');
    else
        warning('gn_main:MissionSequence','Observed phase sequence: %s', ...
            mat2str(gnPhaseValue(gnChanged).'));
    end
    if gnPhaseValue(end)==6
        gnCompleteIndex = find(gnPhaseValue==6,1);
        fprintf('COMPLETE at %.2f s; logged landed tail %.2f s.\n', ...
            gnPhaseTime(gnCompleteIndex),gnPhaseTime(end)-gnPhaseTime(gnCompleteIndex));
    else
        warning('gn_main:Incomplete','Mission ended in phase %g; inspect guards and target.', ...
            gnPhaseValue(end));
    end
else
    warning('gn_main:MissingPhase','phaseLog not found; completion cannot be verified.');
end

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
    exportgraphics(gnComparisonFigure,[gnResultBase '_states.png'],'Resolution',200);
    fprintf('Saved MAT data and PNG plot in %s\n',gnResultsFolder);
end

%% Local helpers -- keep these at the bottom of this same file
function port = gnOutputPort(mdl,blockName,aliases)
% Resolve a top-level subsystem output by its internal Outport block name.
port = [];
blocks = find_system(mdl,'SearchDepth',1,'MatchFilter', ...
    @Simulink.match.allVariants,'Type','Block','Name',blockName);
if numel(blocks)~=1, return; end
block = blocks{1};
outs = find_system(block,'SearchDepth',1,'MatchFilter', ...
    @Simulink.match.allVariants,'BlockType','Outport');
ports = get_param(block,'PortHandles');
for k = 1:numel(outs)
    name = get_param(outs{k},'Name');
    if any(strcmpi(name,aliases))
        index = str2double(get_param(outs{k},'Port'));
        port = ports.Outport(index);
        return
    end
end
% Stateflow charts may have no visible internal Outport blocks. Their
% external port names can still identify phase and target outputs.
for k = 1:numel(ports.Outport)
    name = get_param(ports.Outport(k),'Name');
    if any(strcmpi(name,aliases))
        port = ports.Outport(k);
        return
    end
end
end

function gnTapOutput(mdl,blockName,aliases,logName)
port = gnOutputPort(mdl,blockName,aliases);
if isempty(port)
    warning('gn_main:PortNotFound', ...
        'Cannot find %s output %s. Log it manually with custom name %s.', ...
        blockName,strjoin(aliases,' / '),logName);
    return
end
% This changes only the logging properties of the output port.
set_param(port,'DataLogging','on','DataLoggingNameMode','Custom', ...
    'DataLoggingName',logName,'DataLoggingDecimateData','off', ...
    'DataLoggingLimitDataPoints','off');
end

function gnPrepareAnimation(mdl)
% Reuse the user's native animation, or add a display branch if absent.
blocks = find_system(mdl,'LookUnderMasks','all','FollowLinks','on', ...
    'MatchFilter',@Simulink.match.allVariants,'Type','Block');
matches = {};
for k = 1:numel(blocks)
    maskType = get_param(blocks{k},'MaskType');
    reference = get_param(blocks{k},'ReferenceBlock');
    if contains(lower(maskType),'uav animation') || ...
            contains(lower(reference),'uav animation')
        matches{end+1} = blocks{k}; %#ok<AGROW>
    end
end
if isempty(matches)
    % Resolve the library block from the installed toolbox, without assuming
    % its internal Utilities folder path.
    try
        load_system('uavlib');
        candidates = find_system('uavlib','LookUnderMasks','all', ...
            'MatchFilter',@Simulink.match.allVariants, ...
            'Type','Block','Name','UAV Animation');
    catch
        candidates = {};
    end
    assert(~isempty(candidates), ...
        ['Add UAV Toolbox > Utilities > UAV Animation to the model first. ' ...
         'Connect Ground State pDisplay through Reshape [1 3] to Translation, ' ...
         'and Airborne Plant eulerRaw through Reshape [1 3] to Rotation. ' ...
         'Set Down, Euler ZYX, and auto-open output figure.']);
    p = gnOutputPort(mdl,'Ground State',{'pDisplay'});
    e = gnOutputPort(mdl,'Airborne Plant',{'eulerRaw','EulerZYX'});
    assert(~isempty(p) && ~isempty(e), ...
        'Animation needs Ground State pDisplay and Airborne Plant eulerRaw. Check output names.');
    animation = [mdl '/Live Mission Animation'];
    assert(getSimulinkBlockHandle(animation)==-1, ...
        'A block named Live Mission Animation already exists; inspect it before retrying.');
    add_block(candidates{1},animation,'Position',[1200 650 1400 750]);
    gnAddAnimationRow(mdl,p,animation,1,'Live Position Row',[1010 650 1110 685]);
    gnAddAnimationRow(mdl,e,animation,2,'Live Euler Row',[1010 715 1110 750]);
    matches = {animation};
end
for k = 1:numel(matches)
    block = matches{k};
    mask = Simulink.Mask.get(block);
    assert(~isempty(mask),'Cannot inspect UAV Animation block mask.');
    params = mask.Parameters;
    autoOpen = false;
    for j = 1:numel(params)
        prompt = lower(params(j).Prompt);
        if contains(prompt,'always open') && contains(prompt,'figure')
            set_param(block,params(j).Name,'on');
            autoOpen = true;
        elseif contains(prompt,'z-axis direction')
            gnSetMaskOption(block,params(j),'down');
        elseif contains(prompt,'rotation format')
            gnSetMaskOption(block,params(j),'euler');
        elseif contains(prompt,'uav type')
            gnSetMaskOption(block,params(j),'multirotor');
        elseif strcmp(strtrim(prompt),'sample time') || strcmp(strtrim(prompt),'sample time:')
            set_param(block,params(j).Name,'0.1');
        end
    end
    if ~autoOpen
        open_system(block);
        warning('gn_main:OlderAnimation', ...
            ['This animation block has no auto-open option. Click Show animation ' ...
             'or Open output figure in its dialog, then rerun gn_main if necessary.']);
    end
end
end

function gnSetMaskOption(block,param,word)
options = param.TypeOptions;
for k = 1:numel(options)
    if contains(lower(options{k}),word)
        set_param(block,param.Name,options{k});
        return
    end
end
error('gn_main:AnimationOption', ...
    'Set %s manually in %s; required option contains "%s".', ...
    param.Prompt,block,word);
end

function gnAddAnimationRow(mdl,source,animation,index,name,position)
reshape = [mdl '/' name];
add_block('simulink/Math Operations/Reshape',reshape, ...
    'OutputDimensionality','Customize','OutputDimensions','[1 3]', ...
    'Position',position);
rp = get_param(reshape,'PortHandles');
ap = get_param(animation,'PortHandles');
add_line(mdl,source,rp.Inport(1),'autorouting','on');
add_line(mdl,rp.Outport(1),ap.Inport(index),'autorouting','on');
end

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
