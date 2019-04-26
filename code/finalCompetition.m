function[dataStore] = finalCompetition(CreatePort,DistPort,TagPort,tagNum,maxTime)
%   dataStore = finalCompetition(CreatePort,DistPort,TagPort,tagNum,maxTime) 
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

map = 'compMap.mat';
mapstruct = importdata(map);
mapdata = mapstruct.map;
sensorOrigin = [0.13 0];
angles = linspace(-27*pi/180,27*pi/180,9);

tic
while toc < maxTime
    
    % READ & STORE SENSOR DATA
    mu = 0;
    sigma = sqrt(0.01);
    [noRobotCount,dataStore]=readStoreSensorData(CreatePort,DistPort,TagPort,tagNum,noRobotCount,dataStore);
    
    dataStore.GPS = [dataStore.GPS;...
        toc dataStore.truthPose(end,2:end) + normrnd(mu,sigma,1,3)];
    
    dvec = dataStore.odometry(end,2);
    phivec = dataStore.odometry(end,3);
    
    % First Round
    if initsw == 0
        newstate = integrateOdom_onestep(dataStore.truthPose(1,2),...
            dataStore.truthPose(1,3), dataStore.truthPose(1,4), 0, 0);
        xt = newstate(1);
        yt = newstate(2);
        thetat = newstate(3);
        dataStore.ekfMu = [toc dataStore.truthPose(1,2:end)];
        dataStore.ekfSigma = [2 0 0;0 2 0;0 0 0.1];
        %5b
                R = 0.01*eye(3);
                Q = 0.1*eye(9); %for 9 point depth
        
        initsw = 1;
        [a, b] = drawparticle(dataStore.ekfMu(end,2),dataStore.ekfMu(end,3),dataStore.ekfMu(end,4));
    % Second Round
    elseif initsw == 1
        newstate = integrateOdom_onestep(dataStore.deadReck(end,2),...
            dataStore.deadReck(end,3), dataStore.deadReck(end,4), dvec, phivec);
        xt = newstate(1);
        yt = newstate(2);
        thetat = newstate(3);
        
        dataStore.ekfMu = [dataStore.ekfMu; toc dataStore.truthPose(2,2:end)];
        dataStore.ekfSigma = cat(3,dataStore.ekfSigma,dataStore.ekfSigma);
        initsw = 2;
    % More Than Three Round    
    else
        newstate = integrateOdom_onestep(dataStore.deadReck(end,2),...
            dataStore.deadReck(end,3), dataStore.deadReck(end,4), dvec, phivec);
        xt = newstate(1);
        yt = newstate(2);
        thetat = newstate(3);
        
        mu_last = dataStore.ekfMu(end,2:end);
        sig_last = dataStore.ekfSigma(:,:,end);
        
        g_handle = @(mubar, u) integrateOdom_onestep(mubar(1),mubar(2),mubar(3),u(1),u(2));
        Gjac_handle = @(mubar, u) GjacDiffDrive(mubar, u(1), u(2));
        
        dlast = dataStore.odometry(end-1,2);
        philast = dataStore.odometry(end-1,3);
        
        ctrlbar = [dlast philast];
        h_handle = @(mutbar) drPredict(mutbar,mapdata,sensorOrigin,angles);
        mu_lastlast = dataStore.ekfMu(end-1,2:end);
        mutbarbar = integrateOdom_onestep(mu_lastlast(1),mu_lastlast(2),...
            mu_lastlast(3),ctrlbar(1),ctrlbar(2));
        Hjac_handle = @(mutbar) HjacDepth(mutbarbar,mutbar,mapdata);
        
        
        ctrl = [dvec phivec];
        zt = dataStore.rsdepth(end,3:end)';
        
        [mu_t, sig_t, kt] = extendedKalmanFilter(h_handle,g_handle,...
            Hjac_handle,Gjac_handle,mu_last,sig_last,ctrl,R,Q,zt);
        dataStore.kttest = cat(3,dataStore.kttest,kt);
        dataStore.ekfMu = [dataStore.ekfMu; toc mu_t'];
        dataStore.ekfSigma = cat(3,dataStore.ekfSigma,sig_t);
    end
    
    delete(a)
    delete(b)
    [a, b] = drawparticle(dataStore.ekfMu(end,2),dataStore.ekfMu(end,3),dataStore.ekfMu(end,4));
    
    
    
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
    elseif done==1 && wall==0 && rotate==1
        SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW );
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
