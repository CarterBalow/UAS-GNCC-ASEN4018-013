% Run this script from the UAV_GNC_Project root.

addpath(fullfile(pwd,'functions'));

addpath(fullfile(pwd,'scripts'));

addpath(fullfile(pwd,'models'));

 

cfg.home = [0;0;0];

cfg.aoiCorner = [6;2];

cfg.aoiSize = [5;5];

cfg.altitude = 1.5;

cfg.hMax = 2;

cfg.vNom = 0.4;

cfg.vMax = 0.5;

cfg.landSpeed = 0.15;

% Use the lower end of the selected cross-track FOV range.
% The camera's wide image dimension must point across the lanes.
cfg.cameraCrossTrackFOV = 75;   % Degrees
cfg.cameraLaneOverlap = 0.20;  % Development assumption: 30%

% Estimate ground coverage using camera height above flat ground.
cfg.cameraFootprintWidth = 2*cfg.altitude*tand(cfg.cameraCrossTrackFOV/2);

% Adjacent passes overlap instead of leaving gaps between footprints.
cfg.laneSpacing = cfg.cameraFootprintWidth*(1-cfg.cameraLaneOverlap);

cfg.ctrlTs = 0.02;

cfg.navTs = 0.02;    % Later change to 0.2.

cfg.animTs = 0.1;

cfg.mass = 1.5;      % Illustrative; replace with project mass.

cfg.g = 9.81;

cfg.kPos = 0.8;

cfg.kVel = 2.5;

cfg.aMax = 0.6;

cfg.tiltMax = deg2rad(10);

cfg.wpTol = 0.08;

cfg.homeTol = 0.05;

cfg.arrivalSpeed = 0.04;

cfg.lookahead = 0.5;

cfg.transitionRadius = 0.08;

cfg.stopTime = 600;

cfg.lla0 = [40.0,-105.0,1600]; % Arbitrary WGS84 origin, not a survey.

cfg.useGPS = false;

cfg.gpsPosSigma = 0;  % Start zero-noise for coordinate tests.

cfg.gpsVelSigma = 0;

 

assert(cfg.altitude>0 && cfg.altitude<=cfg.hMax);

assert(cfg.vNom>0 && cfg.vNom<=cfg.vMax);

assert(cfg.landSpeed>0 && cfg.landSpeed<=cfg.vMax);

assert(all(cfg.aoiSize>0) && cfg.laneSpacing>0);

assert(abs(cfg.navTs/cfg.ctrlTs-round(cfg.navTs/cfg.ctrlTs))<1e-9);

 

[searchW,homeAir,entryPoint,lastPoint] = gn_buildSearch(cfg);

home = cfg.home;

C = [cfg.kPos;cfg.vNom;cfg.vMax;cfg.landSpeed;cfg.aMax; ...

     cfg.mass;cfg.g;cfg.tiltMax;cfg.wpTol;cfg.homeTol; ...

     cfg.arrivalSpeed];

 

ctrlNames = {'Roll','Pitch','YawRate','Thrust'};

for j=1:4

    ce(j) = Simulink.BusElement;

    ce(j).Name = ctrlNames{j};

    ce(j).Dimensions = 1;

    ce(j).DataType = 'double';

end

GNControlBus = Simulink.Bus;

GNControlBus.Elements = ce;

ge = Simulink.BusElement;

ge.Name='Gravity'; ge.Dimensions=1; ge.DataType='double';

GNEnvironmentBus = Simulink.Bus;

GNEnvironmentBus.Elements = ge;

 

nativeModel = multirotor;

disp(nativeModel.Configuration);

Rcheck=angle2dcm(pi/2,0,0,'ZYX');

assert(norm(Rcheck*[1;0;0]-[0;-1;0])<1e-9);

disp(table(searchW(:,1),searchW(:,2),-searchW(:,3), 'VariableNames',{'North_m','East_m','Altitude_m'}));



% mission top down plot
% figure;
% 
% plot(searchW(:,2),searchW(:,1),'o-'); hold on;
% 
% rectangle('Position',[cfg.aoiCorner(2),cfg.aoiCorner(1), cfg.aoiSize(2),cfg.aoiSize(1)],'EdgeColor','r');
% 
% plot(home(2),home(1),'ks','MarkerFaceColor','k');
% 
% axis equal; grid on; xlabel('East (m)'); ylabel('North (m)');
% 
% title('Search geometry and home');