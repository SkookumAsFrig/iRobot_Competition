function[dataStore] = motionControl_PF(CreatePort,DistPort,TagPort,tagNum,maxTime)
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
    'deadReck', [], ...
    'ekfMu', [], ...
    'ekfSigma', [],...
    'particles', [],...
    'debugparticles', []);


% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.
noRobotCount = 0;
%switches for pseudo-state machine
done = 1;
%done for finishing backing up
wall = 0;
%wall for hitting a wall
rotate = 1;
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

initsw = 0;
dvec = 0;
phivec = 0;

numpart = 300;

map = 'compMap.mat';
mapstruct = importdata(map);
mapdata = mapstruct.map;
beaconmat = mapstruct.beaconLoc;
sensorOrigin = [0.13 0];
angles = linspace(27*pi/180,-27*pi/180,9);
spinsw = 0;

[noRobotCount,dataStore]=readStoreSensorData(CreatePort,DistPort,TagPort,tagNum,noRobotCount,dataStore);

tic
while toc < maxTime
    % READ & STORE SENSOR DATA
    
    [q,~] = size(dataStore.beacon);
    [noRobotCount,dataStore]=readStoreSensorData(CreatePort,DistPort,TagPort,tagNum,noRobotCount,dataStore);
    
    dvec = dataStore.odometry(end,2);
    phivec = dataStore.odometry(end,3);
    
    if initsw == 0
        dataStore.beacon = [0,0,-1,0,0,0];   
        
        oriPose = dataStore.truthPose(1,2:4);
        
        initsw = 1;
        %% PF
        
        xpart = -5*rand(numpart,1);
        ypart = 10*rand(numpart,1)-5;
        thepart = 0.4*rand(numpart,1)-0.2;
        wi = ones(numpart,1)/numpart;
        dataStore.particles = [xpart ypart thepart wi];
        [a, b] = drawparticle(mean(dataStore.particles(:,1,end)),mean(dataStore.particles(:,2,end)),mean(dataStore.particles(:,3,end)));
    else
        
        ctrl = [dvec phivec];
        zt = [dataStore.rsdepth(end,3:end) dataStore.beacon(end,3:5)]';
        
        %% PF
        M_init = dataStore.particles(:,:,end);
        ctrl_handle = @(mubar,ubar) integrateOdom_onestep_wnoise(mubar(1),...
            mubar(2),mubar(3),ubar(1),ubar(2));
        w_handle = @(xtpred,meas) findWeight(xtpred,mapdata,sensorOrigin,...
            angles,meas,beaconmat);
        o_handle = @(PSet) offmap_detect(PSet,mapdata);
        reinit_handle = @() resample(mapdata,numpart);
        [M_final,Mt]= particleFilter(M_init,ctrl,zt,ctrl_handle,w_handle,o_handle,reinit_handle);
        dataStore.particles = cat(3,dataStore.particles,M_final);
        dataStore.debugparticles = cat(3,dataStore.debugparticles,Mt);
        
    end
    [p,~] = size(dataStore.beacon);
    if p==q
        dataStore.beacon = [dataStore.beacon; [0,0,-1,0,0,0]];
    end
    disp(dataStore.beacon(end,3));
    
    delete(a)
    delete(b)
    [a, b] = drawparticle(mean(dataStore.particles(:,1,end)),mean(dataStore.particles(:,2,end)),mean(dataStore.particles(:,3,end)));
    
    % CONTROL FUNCTION (send robot commands)
    
    % Set forward velocity
    fwdVel = 0.2;
    angVel = 0;
    [cmdV,cmdW] = limitCmds(fwdVel,angVel,0.49,0.13);
    % Set rotate velocity
    fwdVel2 = 0;
    angVel2 = -0.2;
    [cmdV2,cmdW2] = limitCmds(fwdVel2,angVel2,0.49,0.13);
    
    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(CreatePort, 0,0);
    elseif spinsw<2
        SetFwdVelAngVelCreate(CreatePort, cmdV2, cmdW2 );
        if(dataStore.truthPose(end,4) - oriPose(3) > 1.5*pi)
            spinsw = spinsw+1;
        end
    elseif done==1 && wall==0 && rotate==1
        SetFwdVelAng
        VelCreate(CreatePort, cmdV, cmdW );
        %go forward unless it hits something all around
        [BumpRight,BumpLeft,WheDropRight,WheDropLeft,WheDropCaster,BumpFront] = BumpsWheelDropsSensorsRoomba(CreatePort);
        if BumpFront || BumpRight || BumpLeft
            %go into back up state
            wall = 1;
            done = 0;
            rotate = 0;
        end
    elseif done==0 && wall==1 && rotate==0
        if distanceticker == 0
            %oneshot to set back up distance reference
            Lastpoint = dataStore.truthPose(end,2:3);
            distanceticker = 1;
        else
            %calculate distance by sqrt(deltax^2+deltay^2)
            Distanceout = norm(dataStore.truthPose(end,2:3) - Lastpoint);
        end
        
        if Distanceout < 0.25
            %back up if not there yet
            SetFwdVelAngVelCreate(CreatePort, -cmdV, cmdW );
        else
            %if reached, stop and go into rotate state
            SetFwdVelAngVelCreate(CreatePort, 0, 0 );
            Distanceout = 0;
            distanceticker = 0;
            done = 1;
            wrap = 0;
            Lastangle = dataStore.truthPose(end,4);
            %record angle for turn reference
        end
    elseif done==1 && wall==1 && rotate==0
        test = dataStore.truthPose(end,4) - Lastangle;
        %test for wrap around
        if abs(test)>pi
            %pi is arbitrary threshold, tbh anything over pi/6 works
            wrap = 2*pi;
        end
        Angleout = dataStore.truthPose(end,4) - Lastangle - wrap;
        %this algo fixes wraparound issues
        
        if abs(Angleout) < pi/6
            %keep turning if not there yet
            SetFwdVelAngVelCreate(CreatePort, cmdV2, cmdW2 );
        else
            %stop if reached, go into forward state and reset all temp
            %variables
            SetFwdVelAngVelCreate(CreatePort, 0, 0 );
            Angleout = 0;
            rotateticker = 0;
            rotate = 1;
            wall = 0;
            wrap = 0;
        end
    end
    
    pause(0.1);
end

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(CreatePort, 0,0 );
