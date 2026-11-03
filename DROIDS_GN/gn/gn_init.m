% Mission config (cfg from mission_params), planned mission, and Simulink buses. Works from any folder.

gn_dir = fileparts(mfilename('fullpath'));
addpath(gn_dir, fullfile(fileparts(gn_dir), 'power'), fileparts(gn_dir));

cfg = mission_params();   % all inputs: ../mission_params.m

[searchW,homeAir,entryPoint,lastPoint,mission,plan] = gn_build_mission(cfg);

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