function[dataStore] = finalCompetition_biRRT_LAB(CreatePort,DistPort,TagPort,tagNum,maxTime)
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
    'timebeacon', [], ...
    'deadReck', [], ...
    'ekfMu', [], ...
    'ekfSigma', [],...
    'particles', []);

%% Initialize Condition Variables
% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.
noRobotCount = 0;
done = 1;   %switches for pseudo-state machine
wall = 0;   %done for finishing backing up
rotate = 1; %wall for hitting a wall
%Lastpoint to reference for distance calculation
Lastpoint = zeros(1,2);
%Lastangle to reference for angle calculation
Lastangle = 0;
%tickers for one time setting reference
distanceticker = 0;
rotateticker = 0;
%distance and angle values
Distanceout = 0;
Angleout = 0;

initsw = 0; % state switch flag

spinsw = 0; % inner state switch flag {0---Star spin; 
                                      %1---Finish spin twice; 
                                      %2---Stare at beacon done}
startTimer = 0; % 3-second timer launch flag

seeBeacInd = 1;

%% Initialize Configuration Variables
sensorOrigin = [0.13 0];
angles = linspace(27*pi/180,-27*pi/180,9);
% odometry vector
dvec = 0;
phivec = 0;
% particle number
numpart = 125;

partTraj = [];

%% Load Map & Beacon Information
map = 'compMap_mod.mat';
mapstruct = importdata(map);
mapdata = mapstruct.map;
beaconmat = mapstruct.beaconLoc;
[beaconsize,~] = size(beaconmat);
mapdata = mapstruct.map;
[mapsize,~] = size(mapdata);
maxX = max([max(mapdata(:,1)) max(mapdata(:,3))]);
maxY = max([max(mapdata(:,2)) max(mapdata(:,4))]);
minX = min([min(mapdata(:,1)) min(mapdata(:,3))]);
minY = min([min(mapdata(:,2)) min(mapdata(:,4))]);
xrange = (maxX-minX);
yrange = (maxY-minY);
beconID = beaconmat(:,1);

% goalp = [2.33 0.82];
goalp = [2.16 -1.03];
% goalp = [-2.43 -0.5];

% goalp = [2 -1]; SUCCEED
% goalp = [0 -1.2]; SUCCEED
% goalp = [-2 1]; FAILED
map = 'compMap_mod.mat';

%goalp = [3 3];
%goalp = [1.5 3.5];
% goalp = [-3 3.5];
% map = 'compMap_big.mat';

limits = [minX minY maxX maxY];
%sampling_handle = @(limits,lastind) lowdisp_resample(limits,lastind);
sampling_handle = @(limits) uniformresample(limits);
radius = 0.3;
stepsize = 0.4;
oneshot = 0;

%constants to initialize
epsilon = 0.2;
closeEnough = 0.1;
gotopt = 1;
reached = 0;
alph = 20;
%last is to stop robot when last waypoint is reached
last = 0;
DRweight = 1;

tic

figure(1)
hold on
for j=1:mapsize
    plot([mapdata(j,1) mapdata(j,3)],[mapdata(j,2) mapdata(j,4)],'LineWidth',2,'Color','k','HandleVisibility','off')
end
for e=1:beaconsize
    plot(beaconmat(e,2),beaconmat(e,3),'rp','MarkerFaceColor','r')
    text(beaconmat(e,2),beaconmat(e,3),num2str(beaconmat(e,1)))
end
xlim([-3, 3]);
ylim([-2.5, 2.5]);

hold on
% READ & STORE SENSOR DATA
[noRobotCount,dataStore]=readStoreSensorData(CreatePort,DistPort,TagPort,tagNum,noRobotCount,dataStore);
deadreck = dataStore.truthPose(1,2:4);
% deadreck = [minX+xrange/2 minY+yrange/2 0];
noiseprofile = [sqrt(0.3) sqrt(0.3) sqrt(0.3)];

%% RUNNING LOOP
while toc < maxTime && last~=1  % WITHIN SETTING TIME & LAST WAYPOINT IS NOT REACHED
    
    % CHECK DATA SIZE BEFORE READ SENSOR DATA
    [q,~] = size(dataStore.beacon);
    [noRobotCount,dataStore]=readStoreSensorData(CreatePort,DistPort,TagPort,tagNum,noRobotCount,dataStore);
    % CHECK DATA SIZE AGAIN
    [p,~] = size(dataStore.beacon);
    
    % BEACON DATA ALIGN
    if p==q % NO BEACON SEEN
        dataStore.timebeacon = [dataStore.timebeacon; repmat([-1,0,0],1,beaconsize)];
        %disp("cannot see beacon")
    else % SEE BEACON
        newpsize = p-q-1;
        newdata = reshape(dataStore.beacon(end-newpsize:end,3:5)',1,[]);
        newdata = padarray(newdata,[0 3*beaconsize-length(newdata)],-1,'post');
        dataStore.timebeacon = [dataStore.timebeacon; newdata];
        %disp("I see beacon!")
    end
    
    % GET ODOMETRY DATA
    dvec = dataStore.odometry(end,2);
    phivec = 1.05*dataStore.odometry(end,3);
    
    if dataStore.timebeacon(end,1)~=-1
        seeBeacInd = seeBeacInd+1;
    else
        seeBeacInd = 1;
    end
    
    if seeBeacInd>5
        xi = mean(dataStore.particles(:,1,end-1));
        yi = mean(dataStore.particles(:,2,end-1));
        thetai = mean(dataStore.particles(:,3,end-1));
        seeBeacInd = 1;
    else
        xi = deadreck(end,1);
        yi = deadreck(end,2);
        thetai = deadreck(end,3);
    end
    
    newstate = integrateOdom_onestep(xi, yi, thetai, dvec, phivec);
    deadreck = [deadreck; newstate'];
    
    %% STATE 0: INIT
    if initsw == 0
        % init beacon
        dataStore.beacon = [0,0,-1,0,0,0];
        % init current robot direction
        oriPose = dataStore.odometry(1,3);
        turnSum = oriPose;
        spinTwo = 0;
        % init PF
        xpart = xrange*rand(numpart,1)+minX;
        ypart = yrange*rand(numpart,1)+minY;
        thepart = 2*pi*rand(numpart,1);
        wi = ones(numpart,1)/numpart;
        dataStore.particles = [xpart ypart thepart wi];
        % plot particles set
        %[a, b] = drawparticle(mean(dataStore.particles(:,1,end)),mean(dataStore.particles(:,2,end)),mean(dataStore.particles(:,3,end)));
        
        % to STATE 2
        initsw = 1;
    %% STATE 2:
    else
        ctrl = [dvec phivec];
        zt = [dataStore.rsdepth(end,3:end) dataStore.timebeacon(end,:) deadreck(end,:)]';
        
        %% PF
        M_init = dataStore.particles(:,:,end);
        ctrl_handle = @(mubar,ubar) integrateOdom_dynamic(mubar(1),...
            mubar(2),mubar(3),ubar(1),ubar(2),noiseprofile);
        w_handle = @(xtpred,meas) findWeight(xtpred,mapdata,sensorOrigin,...
            angles,meas,beaconmat);
        o_handle = @(PSet) offmap_detect(PSet,mapdata);
        reinit_handle = @() resample(mapdata,numpart);
        [M_final,Mt]= particleFilter(M_init,ctrl,zt,ctrl_handle,w_handle,o_handle,reinit_handle);
        dataStore.particles = cat(3,dataStore.particles,M_final);        
    end
    
    xpartmean = mean(dataStore.particles(:,1,end));
    ypartmean = mean(dataStore.particles(:,2,end));
    tpartmean = mean(dataStore.particles(:,3,end));
    plot(dataStore.truthPose(1:end,2),dataStore.truthPose(1:end,3),'b')
    plot(deadreck(1:end,1),deadreck(1:end,2),'g')
    if ~(spinsw <= 1)
        partTraj = [partTraj; [xpartmean ypartmean tpartmean]];
        plot(partTraj(:,1),partTraj(:,2),'r')
    end
    %     [h, i] = drawparticlestar_green(deadreck(end,1),deadreck(end,2),deadreck(end,3));
    
    %     delete(a);
    %     delete(b);
    %[a, b] = drawparticle(mean(dataStore.particles(:,1,end)),mean(dataStore.particles(:,2,end)),mean(dataStore.particles(:,3,end)));
    
    % CONTROL FUNCTION (send robot commands)
    
    % Set rotate velocity
    fwdVel2 = 0;
    angVel2 = -0.1;
    [cmdV2,cmdW2] = limitCmds(fwdVel2,angVel2,0.49,0.13);
    
    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(CreatePort, 0,0);
    elseif spinsw <= 1
        % START SPINNING
        if spinsw == 0
            noiseprofile = [sqrt(0.3) sqrt(0.3) sqrt(0.15)];
            DRweight = 0;
            SetFwdVelAngVelCreate(CreatePort, cmdV2, cmdW2 );
            turnSum = turnSum + dataStore.odometry(end,3);
            % 360 degree spinning
            if abs(turnSum) >= 2*pi
                spinTwo = 1;
            end
            if(spinTwo == 1)
                spinsw = spinsw+1;  % spinsw: 0 -> 1
            end
        % KEEP SPINNING
        else
            noiseprofile = [sqrt(0.25) sqrt(0.25) sqrt(0.25)];
            % when a beacon in view then stop
            if dataStore.timebeacon(end,1) ~= -1
                disp("stopped and I see beacon!")
                if startTimer == 0
                    SetFwdVelAngVelCreate(CreatePort, 0, 0 );
                    timer1 = toc;
                    startTimer = 1;
                else
                    if toc - timer1 > 6
                        spinsw = spinsw+1;  % spinsw: 1 -> 2
                        noiseprofile = [sqrt(0.005) sqrt(0.005) sqrt(0.1)];
                        DRweight = 3;
                    end
                end
            end
            
        end
    else
        if oneshot==0
            %dataStore.timebeacon = [0,0,-1,0,0,0];
            figure(2)
            hold on
            for j=1:mapsize
                a=plot([mapdata(j,1) mapdata(j,3)],[mapdata(j,2) mapdata(j,4)],'LineWidth',2,'Color','k','HandleVisibility','off');
            end
            axis equal
            nowp = deadreck(end,1:2);
            [newV,newconnect_mat,cost,path,pathpoints,expath,expoint,expath2,expoint2] = buildBIRRT(map,limits,sampling_handle,nowp,goalp,stepsize,radius);
            
            d=plot(pathpoints(:,1),pathpoints(:,2),'mo-','LineWidth',2,'MarkerFaceColor',[1 0 1]);
            e=plot(nowp(1),nowp(2),'ko','MarkerFaceColor',[1 0 0]);
            f=plot(goalp(1),goalp(2),'ko','MarkerFaceColor',[0 1 0]);
            
            oneshot=1;
            waypoints = pathpoints;
            [m,~] = size(waypoints);
        end
        
        % Get current pose
        
        x = xpartmean;
        y = ypartmean;
        theta = tpartmean;
        %         x = deadreck(end,1);
        %         y = deadreck(end,2);
        %         theta = deadreck(end,3);
        
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
        [cmdV,cmdW] = limitCmds(vout,wout,0.1,0.13);
        
        % PRINT-----------------------------------
        disp(['cmdV is ' num2str(cmdV)])
        disp(['cmdW is ' num2str(cmdW)])
        % PRINT-----------------------------------
        
        if last==1
            %stop robot if last one reached
            cmdV=0;
            cmdW=0;
        end
        
        % check NaN
        if isnan(cmdV)
            cmdV = 0;
        end       
        if isnan(cmdW)
            cmdW = 0;
        end
        
        SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW);
    end
    % PLOT-----------------------------------
    figure(1)
    hold on
    [x, z, c, v] = drawparticlestar(dataStore.truthPose(end,2),dataStore.truthPose(end,3),dataStore.truthPose(end,4));
    [h, i] = drawparticlestar_red(xpartmean,ypartmean,tpartmean);
    
    pause(0.1);
    
    delete(h);
    delete(i);
    delete(c);
    delete(v);
    delete(x);
    delete(z);
    % PLOT-----------------------------------
end

% PLOT-----------------------------------
figure(2)
% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(CreatePort, 0,0 );
g = plot(dataStore.truthPose(:,2),dataStore.truthPose(:,3),'b');
legend([a expath expoint expath2 expoint2 d e f g],'Map','Start Search Tree Edges',...
    'Start Search Tree Nodes','Goal Search Tree Edges','Goal Search Tree Nodes',...
    'Final Solution Path','Starting Point','Goal Point','Actual Trajectory','Location','northeastoutside')
title('birrtPlanner Search Tree, Solution Path and Robot Trajectory')
xlabel('Global X')
ylabel('Global Y')
% PLOT-----------------------------------
