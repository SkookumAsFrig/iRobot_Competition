function[dataStore] = birrtPlanner(CreatePort,DistPort,TagPort,tagNum,maxTime)
% backupBump: Drives robot forward at constant velocity until it bumps
% into somthing. If a bump sensor is triggered, command the robot to back
% up 0.25m and turn clockwise 30 degs, before continuing to drive forward
% again. Saves a datalog.
%
%   dataStore = backupBump(CreatePort,DistPort,TagPort,tagNum,maxTime) runs
%
%   INPUTStype
%       CreatePort  Create port object (get from running RoombaInit)
%       DistPort    Depth ports object (get from running CreatePiInit)
%       TagPort      Tag ports object (get from running CreatePiInit)
%       tagNum      robot number for overhead localization
%       maxTime     max time to run program (in seconds)
%
%   OUTPUTS
%       dataStore   struct containing logged data

%
%   NOTE: Assume differential-drive robot whose wheels turn at a constant
%         rate between sensor readings.
%
%   Cornell University
%   MAE 5180: Autonomous Mobile Robots

% Set unspecified inputs
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    DistPort = CreatePort;
    TagPort = CreatePort;
    tagNum = CreatePort;
    maxTime = 500;
elseif nargin < 3
    TagPort = CreatePort;
    tagNum = CreatePort;
    maxTime = 500;
elseif nargin < 4
    tagNum = CreatePort;
    maxTime = 500;
elseif nargin < 5
    maxTime = 500;
end


% declare dataStore as a global variable so it can be accessed from the
% workspace even if the program is stopped
global dataStore;

% initialize datalog struct (customize according to needs)
dataStore = struct('truthPose', [],...
    'odometry', [], ...
    'rsdepth', [], ...
    'bump', [], ...
    'beacon', [], ...
    'timebeacon', []);


% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.
noRobotCount = 0;

goalp = [-2.5 1];
map = 'compMap.mat';

%goalp = [3 3];
%goalp = [1.5 3.5];
% goalp = [-3 3.5];
% map = 'compMap_big.mat';

mapstruct = importdata(map);
mapdata = mapstruct.map;
[l,~] = size(mapdata);
xmin = min(min([mapdata(:,1) mapdata(:,3)]));
ymin = min(min([mapdata(:,2) mapdata(:,4)]));
xmax = max(max([mapdata(:,1) mapdata(:,3)]));
ymax = max(max([mapdata(:,2) mapdata(:,4)]));
limits = [xmin ymin xmax ymax];
%sampling_handle = @(limits,lastind) lowdisp_resample(limits,lastind);
sampling_handle = @(limits) uniformresample(limits);
radius = 0.2;
stepsize = 0.2;
oneshot = 0;

%constants to initialize
epsilon = 0.1;
closeEnough = 0.1;
gotopt = 1;
reached = 0;
alph = 20;
%last is to stop robot when last waypoint is reached
last = 0;
[beaconsize,~] = size(mapstruct.beaconLoc);

tic
while toc < maxTime && last~=1
    [q,~] = size(dataStore.beacon);
    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(CreatePort,DistPort,TagPort,tagNum,noRobotCount,dataStore);
    
    % CONTROL FUNCTION (send robot commands)

    [p,~] = size(dataStore.beacon);
    if p==q
        dataStore.timebeacon = [dataStore.timebeacon; repmat([-1,0,0],1,beaconsize)];
        disp("cannot see beacon")
    else
        newpsize = p-q-1;
        newdata = reshape(dataStore.beacon(end-newpsize:end,3:5)',1,[]);
        newdata = padarray(newdata,[0 3*beaconsize-length(newdata)],-1,'post');
        dataStore.timebeacon = [dataStore.timebeacon; newdata];
        disp("I see beacon!")
    end
    
    if oneshot==0
        %dataStore.timebeacon = [0,0,-1,0,0,0];
        figure
        hold on
        for j=1:l
            a=plot([mapdata(j,1) mapdata(j,3)],[mapdata(j,2) mapdata(j,4)],'LineWidth',2,'Color','k','HandleVisibility','off');
        end
        axis equal
        nowp = dataStore.truthPose(end,2:3);
        [newV,newconnect_mat,cost,path,pathpoints,expath,expoint,expath2,expoint2] = buildBIRRT(map,limits,sampling_handle,nowp,goalp,stepsize,radius);
        
        d=plot(pathpoints(:,1),pathpoints(:,2),'mo-','LineWidth',2,'MarkerFaceColor',[1 0 1]);
        e=plot(nowp(1),nowp(2),'ko','MarkerFaceColor',[1 0 0]);
        f=plot(goalp(1),goalp(2),'ko','MarkerFaceColor',[0 1 0]);
        
        oneshot=1;
        waypoints = pathpoints;
        [m,~] = size(waypoints);
    end
    
    % Get current pose
    x = dataStore.truthPose(end,2);
    y = dataStore.truthPose(end,3);
    theta = dataStore.truthPose(end,4);
    
    if reached==1 && gotopt~=m
        %if reached current waypoint, increment index and reset reached
        gotopt = gotopt+1;
        reached = 0;
    elseif gotopt==m && reached==1
        %if last one reached, flag last
        last = 1;
    end
    %run visitWaypoints
    [vout,wout,reached] = visitWaypoints(waypoints,gotopt,closeEnough,epsilon, alph, x, y, theta);
    [cmdV,cmdW] = limitCmds(vout,wout,0.2,0.05);
    
    if last==1
        %stop robot if last one reached
        cmdV=0;
        cmdW=0;
    end
    
    SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW );
    
    pause(0.1);
end

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(CreatePort, 0,0 );
g = plot(dataStore.truthPose(:,2),dataStore.truthPose(:,3),'b');
legend([a expath expoint expath2 expoint2 d e f g],'Map','Start Search Tree Edges',...
    'Start Search Tree Nodes','Goal Search Tree Edges','Goal Search Tree Nodes',...
    'Final Solution Path','Starting Point','Goal Point','Actual Trajectory','Location','northeastoutside')
title('birrtPlanner Search Tree, Solution Path and Robot Trajectory')
xlabel('Global X')
ylabel('Global Y')

