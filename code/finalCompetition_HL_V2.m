function[dataStore] = finalCompetition_HL_V2(CreatePort,DistPort,TagPort,tagNum,maxTime)
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
global wpts_go
global waypoints
global robotestimate
global currwp

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
    'deadreck', [],...
    'particles', []);

%% Load Map File Information
% map
map = 'compMap_mod.mat';
mapstruct = importdata(map);
mapdata = mapstruct.map;
[mapsize,~] = size(mapdata);
maxX = max([max(mapdata(:,1)) max(mapdata(:,3))]);
maxY = max([max(mapdata(:,2)) max(mapdata(:,4))]);
minX = min([min(mapdata(:,1)) min(mapdata(:,3))]);
minY = min([min(mapdata(:,2)) min(mapdata(:,4))]);
xrange = (maxX-minX);
yrange = (maxY-minY);
limits = [minX minY maxX maxY];

% beacon
beaconmat = mapstruct.beaconLoc;
[beaconsize,~] = size(beaconmat);
beconID = beaconmat(:,1);

% waypoints
waypoints = mapstruct.waypoints;
wptsNum = size(waypoints,1);
wallMidpoints = findOptWallMidpoint(mapstruct.optWalls);
wallMptsNum = size(wallMidpoints,1);
wallTwinPts = findOptWallTwinPts(mapstruct.optWalls,0.3);
twinPtsNum = size(wallTwinPts,1);
ECwaypoints = mapstruct.ECwaypoints;
ECwptsNum = size(ECwaypoints,1);
%% All Waypoints Set
wpts_combo = [waypoints, ones(wptsNum,1);
    ECwaypoints, 2*ones(ECwptsNum,1);
    wallTwinPts, [3:twinPtsNum+2]'];
wpts_go = wpts_combo;
% goalp = [2.33 0.82;
%          2.16 -1.03;
%         -2.43 -0.5];


%% Initialize Condition Variables
% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.
noRobotCount = 0;

% sensor data process
seeBeacInd = 0; % beacon continue seen
seeTimes = 2;
% sampling state machine
initsw = 0; % state switch flag
% control state machine
spinsw = 0; % state switch flag {0---Star spin;
%1---Finish spin twice;
%2---Stare at beacon done}
startTimer = 0; % 3-second timer launch flag
stareTime_L = 4;
stareTime_S = 2;

%% Initialize Configuration
% Bump
realWall = 0;
backTime = 0;
backupFlag = 0;
% RSDepth
sensorOrigin = [0.11 0.06];
angles = linspace(27*pi/180,-27*pi/180,9);
% Odometry
dvec = 0;
phivec = 0;
%% PF
partTraj = [];
eachnumpart = 60;
numpart = eachnumpart*wptsNum;
% RRT
sampling_handle = @(limits) uniformresample(limits);
radius = 0.25;
stepsize = 0.3;
rrtplan = 0;
%% Visit waypoints
epsilon = 0.2; %0.6
closeEnough = 0.35;
gotopt = 1;
reached = 0;
alph = 20;
last = 0;   % stop robot when last waypoint is reached
finishAll = 0;
interrupt = 0;  % on the way to waypoint if bumped, not need to remove waypoints

DRweight = 0;

%For initialization wait second stage time
inititer = 0;

 arrived = 1;

%% Back & Turn
cmdV_back = -0.1;
cmdW_turn = -0.2;
backStart = 0;
turnStart = 0;
stillBump = [];

accum_initw = zeros(wptsNum,1);
spinAgain = 0;

% PLOT MAP & BEACONS
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
drawnow

% READ & STORE SENSOR DATA
[noRobotCount,dataStore]=readStoreSensorData(CreatePort,DistPort,TagPort,tagNum,noRobotCount,dataStore);
% deadreck = dataStore.truthPose(1,2:4);
dataStore.deadreck = [minX+xrange/2 minY+yrange/2 0];
noiseprofile = [sqrt(0.1) sqrt(0.1) sqrt(0.5)];
tic
%% RUNNING LOOP
while toc < Inf && finishAll~=1  % WITHIN SETTING TIME & LAST WAYPOINT IS NOT REACHED
    %% ########################################################## %%
    %% ###############SENSOR DATA PROCESS BEGIN################## %%
    %% ########################################################## %%
    %% SENSOR DATA PROCESS
    
    % CHECK BEACON SEEN OR NOT
    [q,~] = size(dataStore.beacon);
    [noRobotCount,dataStore]=readStoreSensorData(CreatePort,DistPort,TagPort,tagNum,noRobotCount,dataStore);
    [p,~] = size(dataStore.beacon);
    
    % ALIGNED BEACON DATA
    if p==q % SEE NO BEACON OR NO MOVE SEE SAME BEACON TWICE
        if p == 0 || p == 1
            dataStore.timebeacon = [dataStore.timebeacon; repmat([-1,0,0],1,beaconsize)];
            disp("no beacon!")
        else
            if dataStore.beacon(end,3) == dataStore.beacon(end-1,3)
                newpsize = 1;
                newdata = reshape(dataStore.beacon(end-newpsize:end,3:5)',1,[]);
                newdata = padarray(newdata,[0 3*beaconsize-length(newdata)],-1,'post');
                dataStore.timebeacon = [dataStore.timebeacon; newdata];
                disp("same beacon!")
            else
                dataStore.timebeacon = [dataStore.timebeacon; repmat([-1,0,0],1,beaconsize)];
                disp("no beacon!")
            end
        end
    else % SEE BEACON
        newpsize = p-q-1;
        newdata = reshape(dataStore.beacon(end-newpsize:end,3:5)',1,[]);
        newdata = padarray(newdata,[0 3*beaconsize-length(newdata)],-1,'post');
        dataStore.timebeacon = [dataStore.timebeacon; newdata];
        disp("I see beacon!")
    end
    
    % CHECK IF REALLY SEE BEACON(> SEETIMES)
    if dataStore.timebeacon(end,1)~=-1
        seeBeacInd = seeBeacInd+1;
    else
        seeBeacInd = 0;
    end
    
    if seeBeacInd> seeTimes
        xi = mean(dataStore.particles(:,1,end-1));
        yi = mean(dataStore.particles(:,2,end-1));
        thetai = mean(dataStore.particles(:,3,end-1));
        seeBeacInd = 0;
    else
        xi = dataStore.deadreck(end,1);
        yi = dataStore.deadreck(end,2);
        thetai = dataStore.deadreck(end,3);
    end
    
    % CHECK BUMP SENSOR
    [realWall] = checkOptionalWalls(dataStore.bump(end,:));
    
    % GET ODOMETRY DATA & store deadreck
    dvec = dataStore.odometry(end,2);
    phivec = dataStore.odometry(end,3);
    newstate = integrateOdom_onestep(xi, yi, thetai, dvec, phivec);
    dataStore.deadreck = [dataStore.deadreck; newstate'];
    % ###############SENSOR DATA PROCESS END################## %
    
    
    %% ########################################################## %%
    %% #####################SAMPLING BEGIN####################### %%
    %% ########################################################## %%
    %% PARTICLES SAMPLING (SAMPLING STATE MACHINE)
    %% STATE 0: INIT PARTICLES SET
    if initsw == 0
        disp("first run");
        % Initialization
        dataStore.beacon = [0,0,-1,0,0,0];      % beacon data
        oriPose = dataStore.odometry(1,3);      % current robot direction
        spinSum = oriPose;                      % sum of angle turned
        spinDone = 0;                           % flag indicates spin finish
        
        % init PF
%         xprep = repmat(waypoints(:,1),1,eachnumpart);
%         xpart = reshape(xprep',[],1);
%         yprep = repmat(waypoints(:,2),1,eachnumpart);
%         ypart = reshape(yprep',[],1);
%         tprep = linspace(2*pi/eachnumpart,2*pi,eachnumpart);
%         thepart = repmat(tprep,1,wptsNum)';
%         wi = ones(numpart,1)/numpart;
%         dataStore.particles = [xpart ypart thepart wi];
        [xpart ypart thepart wi] = initParticleSet(waypoints,eachnumpart,wptsNum,numpart);
        dataStore.particles = [xpart ypart thepart wi];
        
        % To STATE 1
        initsw = 1;
        
    %% STATE 1: PF Waypoints INIT (initsw = 1)
    elseif initsw == 1
        disp("PF init");
        ctrl = [dvec phivec];
        zt = [dataStore.rsdepth(end,3:end) dataStore.timebeacon(end,:) dataStore.deadreck(end,:)]';
        
        %% PF
        M_init = dataStore.particles(:,:,end);
        ctrl_handle = @(mubar,ubar) integrateOdom_dynamic(mubar(1),...
            mubar(2),mubar(3),ubar(1),ubar(2),noiseprofile);
        w_handle = @(xtpred,meas) findWeight_comp(xtpred,mapdata,sensorOrigin,...
            angles,meas,beaconmat,DRweight);
        o_handle = @(PSet) offmap_detect(PSet,mapdata);
        reinit_handle = @() resample(mapdata,numpart);
        [M_final,Mt,initw]= particleFilter_init(M_init,ctrl,zt,ctrl_handle,w_handle,o_handle,reinit_handle,eachnumpart);
        accum_initw = accum_initw+initw;
        disp(accum_initw);
        dataStore.particles = cat(3,dataStore.particles,M_final);
    
    %% STATE 2: RUNNING PF (initsw = 2)
    else
        disp("running PF");
        ctrl = [dvec phivec];
        zt = [dataStore.rsdepth(end,3:end) dataStore.timebeacon(end,:) dataStore.deadreck(end,:)]';
        
        % PF
        M_init = dataStore.particles(:,:,end);
        ctrl_handle = @(mubar,ubar) integrateOdom_dynamic(mubar(1),...
            mubar(2),mubar(3),ubar(1),ubar(2),noiseprofile);
        w_handle = @(xtpred,meas) findWeight_comp(xtpred,mapdata,sensorOrigin,...
            angles,meas,beaconmat,DRweight);
        o_handle = @(PSet) offmap_detect(PSet,mapdata);
        reinit_handle = @() resample(mapdata,numpart);
        [M_final,Mt]= particleFilter(M_init,ctrl,zt,ctrl_handle,w_handle,o_handle,reinit_handle);
        dataStore.particles = cat(3,dataStore.particles,M_final);
    end
    
    %% CALCULATE ROBOT ESTIMATE
    xpartmean = mean(dataStore.particles(:,1,end));
    ypartmean = mean(dataStore.particles(:,2,end));
    tpartmean = mean(dataStore.particles(:,3,end));
    robotestimate = [xpartmean,ypartmean,tpartmean];
    
    % PLOT---truthPose & deadreck & particle traj---
    plot(dataStore.truthPose(1:end,2),dataStore.truthPose(1:end,3),'b')
    plot(dataStore.deadreck(1:end,1),dataStore.deadreck(1:end,2),'g')
    if ~(spinsw <= 1)
        partTraj = [partTraj; [xpartmean ypartmean tpartmean]];
        plot(partTraj(:,1),partTraj(:,2),'r')
    end
    % #####################SAMPLING END####################### %%
    
    
    %% ########################################################## %%
    %% #####################CONTROL BEGIN######################## %%
    %% ########################################################## %%
    %% ROBOT CONTROL(CONTROL STATE MACHINE)
    % Set rotate velocity
    fwdVel = 0;
    angVel = -0.15;
    [cmdV,cmdW] = limitCmds(fwdVel,angVel,0.49,0.13);
    
    %% STATE 0: STOP ROBOT
    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(CreatePort, 0,0);
    %% STATE 1: SPIN BEFORE MOVE
    elseif spinsw <= 1
        % STATE 1.1: START SPINNING (spinsw = 0)
        if spinsw == 0
            disp("waypoint spinning");
            SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW);
            spinSum = spinSum + dataStore.odometry(end,3);
            noiseprofile = [0 0 sqrt(pi/2)];
            DRweight = 0;
            if abs(spinSum) >= 0.1*pi % 360 degree spinning
                spinDone = 1;
                spinSum = 0;
            end
            if(spinDone == 1)   % spinning complete
                spinsw = 1;  % spinsw: 0 -> 1
                spinDone = 0;
            end

        % STATE 1.2: KEEP SPINNING (spinsw = 1)
        else
            % STATE 1.2.1: STOP WHEN A BEACON IN VIEW
            spinSum = spinSum + dataStore.odometry(end,3);
            if dataStore.timebeacon(end,1) ~= -1 %%&& abs(dataStore.timebeacon(end,3)) <= 0.2 % see beacon
                noiseprofile = [sqrt(0.005) sqrt(0.005) sqrt(pi/2)];
                inititer = inititer+1;
                disp("stopped and I see beacon!")
                if startTimer == 0  % start counting staring sec
                    DRweight = 0;
                    SetFwdVelAngVelCreate(CreatePort, 0, 0 );
                    timer1 = toc;
                    startTimer = 1;
                else % staring time up
                    if toc - timer1 > stareTime_L
                        disp("4 seconds passed")
                        spinsw = spinsw + 1;  % spinsw: 1 -> 2
                        noiseprofile = [sqrt(0.002) sqrt(0.002) sqrt(0.01)];
                        DRweight = 0.000005;
                        initsw = 2; % launch PF running
                        seeTimes = 10;
                    elseif toc - timer1 > stareTime_S || inititer > 1
                        disp("first stage passed")
                        DRweight = 0;
                        initsw = 2;
                    end
                end
            elseif abs(spinSum) >= 2*pi % no beacon seen
                if startTimer == 0
                    DRweight = 0;
                    SetFwdVelAngVelCreate(CreatePort, 0, 0 );
                    timer1 = toc;
                    startTimer = 1;
                    
                    [~,Ind] = max(accum_initw);
                    newpartset = dataStore.particles(:,:,end);
                    dataStore.particles(:,:,end) = repmat(newpartset((Ind-1)*eachnumpart+1:Ind*eachnumpart,:),wpsize,1);
                    noiseprofile = [sqrt(0.001) sqrt(0.001) sqrt(0.01)];
                    initsw = 2;
                elseif toc - timer1 > 5
                    onetime = 1;
                    spinsw = 2;  % spinsw: 1 -> 2
                    initsw = 2;
                    DRweight = 0.0001;
                    seeTimes = 10;
                end
            else
                disp("no beacon in view!");
            end
        end
        
        %% STATE 2: MOVE
    elseif spinsw == 2
        %% STATE 2.1: PRM PLANNING
        if rrtplan==0
            disp("planning")
            % PLOT-----------------------------------
            figure(2)
            hold on
            %             for j=1:mapsize
            %                 a=plot([mapdata(j,1) mapdata(j,3)],[mapdata(j,2) mapdata(j,4)],'LineWidth',2,'Color','k','HandleVisibility','off');
            %             end
            %             axis equal
            
            % BE CAREFUL NOT START FROM WAYPOINTS!!!!!!! NEED EXTRA
            % CONDITION
            % eliminate the start point from the whole set
            if size(wpts_go,1) > 1 && interrupt == 0 && arrived == 1
                wpts_go = removePoint(robotestimate,wpts_go);
            end
            
            
            % PRM Planner
            if ~isempty(wpts_go)%nextwaypoint <= size(wpts_go)
                % currwp = wpts_go(nextwaypoint,1:2);
                [currwp,type] = findNeareastPoints(robotestimate(1,1:3),wpts_go,1);
                nowp = robotestimate(1,1:2);
                obstacles = wall2polygon(mapdata,0.27);
                [cost,wpts] = findPath(obstacles,limits,currwp,nowp,mapdata,0.26);
                

%                [newV,newconnect_mat,cost,path,pathpoints,expath,expoint,expath2,expoint2,timeup] ...
%     = buildQuadRRT(map,limits,sampling_handle,nowp,currwp(1,1:2),currwp(2,1:2),currwp(3,1:2),stepsize,radius,3);
               
                
%                 [newV,newconnect_mat,cost,path,pathpoints,expath,expoint,expath2,expoint2] = buildBIRRT(map,limits,sampling_handle,nowp,currwp(1,1:2),stepsize,radius);
%                 disp("heading to:",num2str(currwp(1,1:2)));

                plot(wpts(:,1),wpts(:,2),'mo-','LineWidth',2,'MarkerFaceColor',[1 0 1])
                axis equal
                %                [newV,newconnect_mat,cost,path,pathpoints,expath,expoint,expath2,expoint2,timeup] ...
                %     = buildQuadRRT(map,limits,sampling_handle,nowp,currwp(1,1:2),currwp(2,1:2),currwp(3,1:2),stepsize,radius,3);
                
                
                %                 [newV,newconnect_mat,cost,path,pathpoints,expath,expoint,expath2,expoint2] = buildBIRRT(map,limits,sampling_handle,nowp,currwp(1,1:2),stepsize,radius);
                disp("heading to:?");
                disp(currwp);
                % PLOT-----------------------------------
                %d=plot(pathpoints(:,1),pathpoints(:,2),'mo-','LineWidth',2,'MarkerFaceColor',[1 0 1]);
                e=plot(nowp(1),nowp(2),'ko','MarkerFaceColor',[1 0 0]);
                f=plot(currwp(1),currwp(2),'ko','MarkerFaceColor',[0 1 0]);
                
                rrtplan=1;
                %wpts = pathpoints;
                [m,~] = size(wpts);
                if size(wpts_go,1) == 1
                    finishAll = 1;
                end
            end
        %% STATE 2.2: VISIT WAYPOINTS
        elseif rrtplan == 1
            % Get current pose
            x = xpartmean;
            y = ypartmean;
            theta = tpartmean;
            % bump sensor activated
            if realWall == 1
                % prepare to backup & replan
                rrtplan = 2;
                backStart = 1;  % back up
                turnStart = 0;  % no turn
                backSum = 0;
                turnSum = 0;
                interrupt = 1;
                inititer = 0;
            else
                if reached==1 && gotopt~=m
                    %if reached current waypoint, increment index and reset reached
                    closeEnough = 0.3;
                    gotopt = gotopt+1;
                    reached = 0;
                    if gotopt > m-5
                        closeEnough = 0.15;
                    end
                elseif gotopt==m && reached==1
                    %if last one reached, flag last
                    interrupt = 0;
                    last = 1;
                end
                %run visitWaypoints
                [vout,wout,reached] = visitWaypoints(wpts,gotopt,closeEnough,epsilon, alph, x, y, theta);
                [cmdV,cmdW] = checkCmd(vout,wout,0.15,0.13);
                
                if last==1
                    %stop robot if waypoint reached
                    cmdV=0;
                    cmdW=0;
                    last = 0;
                    initsw = 2;
                    inititer = 0;
                    spinsw = 3; % spin again
                    spinDone = 0;
                    spinAgain = 1;
                    spinSum = dataStore.odometry(end,3); % init sum again
                    startTimer = 0; % reset timer
                    rrtplan= 0; % rrt plan again
                    gotopt = 1;                     % go to point again
                end
                SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW);
            end
            %% STATE 2.3: BACKUP & TURN
        else
            if backStart == 1
                SetFwdVelAngVelCreate(CreatePort, cmdV_back, 0);
                backSum = backSum + dataStore.odometry(end,2);
                if abs(backSum) > 0.2
                    disp("backup done");
                    SetFwdVelAngVelCreate(CreatePort, 0, 0);
                    turnStart = 1;
                    backStart = 0;
                end
            end
            if turnStart == 1
                SetFwdVelAngVelCreate(CreatePort, 0, cmdW_turn);
                turnSum = turnSum + dataStore.odometry(end,3);
                stillBump = [stillBump,realWall];
                backStart = 0;
                if abs(turnSum) > pi/4
                    disp("turn done")
                    SetFwdVelAngVelCreate(CreatePort, 0, 0);
                    if sum(stillBump) > 0 % check bump sensor again
                        backStart = 1;
                        turnStart = 0;
                        disp("bump again!");
                    else
                        turnStart = 0;
                        initsw = 2;
                        spinsw = 2;
                        spinSum = dataStore.odometry(1,3); % init sum again
                        startTimer = 0;
                        rrtplan = 0;
                        inititer = 0;
                        gotopt = 1;
                        disp("replanning");
                    end
                end
            end
        end
    %% STATE 3: SPIN AGAIN (spinsw == 3)
    elseif spinsw == 3
        %% STATE 3.1: START SPINNING
        if spinAgain == 1
            disp("waypoint spinning+");
            SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW);
            spinSum = spinSum + dataStore.odometry(end,3);
            noiseprofile = [0 0 sqrt(pi/2)];
            DRweight = 0;
            if abs(spinSum) >= 0.1*pi % 360 degree spinning
                spinDone = 1;
            end
            if(spinDone == 1)   % spinning complete
                spinAgain = spinAgain+1;  % spinsw: 0 -> 1
                spinDone = 0;
            end
            %% STATE 3.2: KEEP SPINNING (spinAgain = 2)
        else
            spinSum = spinSum + dataStore.odometry(end,3);
            if dataStore.timebeacon(end,1) ~= -1 %%&& abs(dataStore.timebeacon(end,3)) <= 0.15
                noiseprofile = [sqrt(0.005) sqrt(0.005) sqrt(pi/2)];
                inititer = inititer+1;
                if startTimer == 0  % start counting staring sec
                    disp("stopped and I see beacon!+")
                    DRweight = 0;
                    SetFwdVelAngVelCreate(CreatePort, 0, 0 );
                    timer1 = toc;
                    startTimer = 1;
                else % staring time up
                    if toc - timer1 > stareTime_L
                        disp("4 seconds passed+")
                        spinsw = 4;  % spinsw: 3 -> 2
                        noiseprofile = [sqrt(0.002) sqrt(0.002) sqrt(0.01)];
                        DRweight = 0.000005;
                        initsw = 2; % launch PF running
                        seeTimes = 10;
                    elseif toc - timer1 > stareTime_S || inititer > 1
                        disp("first stage passed+")
                        DRweight = 0;
                        initsw = 2;
                    end
                end
            elseif abs(spinSum) >= pi
                disp("180 turned")
                if startTimer == 0
                    DRweight = 0;
                    SetFwdVelAngVelCreate(CreatePort, 0, 0 );
                    timer1 = toc;
                    startTimer = 1;
                    
                    [~,Ind] = max(accum_initw);
                    newpartset = dataStore.particles(:,:,end);
                    dataStore.particles(:,:,end) = repmat(newpartset((Ind-1)*eachnumpart+1:Ind*eachnumpart,:),wptsNum,1);
                    noiseprofile = [sqrt(0.001) sqrt(0.001) sqrt(0.01)];
                    initsw = 2;
                elseif toc - timer1 > 5
                    onetime = 1;
                    spinsw = 4;  % new planning
                    initsw = 2; % always 2
                    DRweight = 0.0001;
                    seeTimes = 10;
                end
            else
                disp("no beacon in view!+");
            end
        end
    %% STATE 4: CALIBRATION (spinsw == 4)
    elseif spinsw == 4
        gap = findEuclideanDistance(robotestimate(1,1:2),currwp);
        if (gap > closeEnough) % replan path for current goal
            disp("calibrate");
            arrived = 0;
        else % replan path for the next goal
            disp("close enough");
            arrived = 1;    
        end
        spinsw = 5;
    %% STATE 5: DETECT WALL
    else
        spinsw = 2;
    end
    % #####################CONTROL END#######################
    
    % PLOT-----------------------------------
    figure(1)
    drawnow
    [x, z, c, v] = drawparticlestar(dataStore.truthPose(end,2),dataStore.truthPose(end,3),dataStore.truthPose(end,4));
    [h, i] = drawparticlestar_red(xpartmean,ypartmean,tpartmean);
    [qw, wq] = drawparticlestar_green(dataStore.deadreck(end,1),dataStore.deadreck(end,2),dataStore.deadreck(end,3));
    
    pause(0.1);
    
    delete(qw);
    delete(wq);
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
