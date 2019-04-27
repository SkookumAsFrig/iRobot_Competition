function[dataStore] = freedriveProgram(CreatePort,DepthPort,TagPort,tagNum,maxTime)
% freedriveProgram: example program to manually drive iRobot Create
% Reads data from sensors, sends robot commands, and saves a datalog.
% 
%   DATASTORE = freedriveProgram(CreatePort,DepthPort,TagPort,tagNum,maxTime)
% 
%   INPUTS
%       CreatePort  Create port object (get from running CreatePiInit)
%       DepthPort   Depth port object (get from running CreatePiInit)
%       TagPort     Tag port object (get from running CreatePiInit)
%       tagNum      robot number for overhead localization
%       maxTime     max time to run program (in seconds)
% 
%   OUTPUTS
%       dataStore   struct containing logged data
% 
%   Cornell University
%   MAE 5180: Autonomous Mobile Robots
%   Lab #3
%   LASTNAME, FIRSTNAME 

% Set unspecified inputs
defaultMaxTime = 500;
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    DepthPort = CreatePort;
    TagPort = CreatePort;
    tagNum = CreatePort;
    maxTime = defaultMaxTime;
elseif nargin < 3
    TagPort = CreatePort;
    tagNum = CreatePort;
    maxTime = defaultMaxTime;
elseif nargin < 4
    tagNum = CreatePort;
    maxTime = defaultMaxTime;
elseif nargin < 5
    maxTime = defaultMaxTime;
end

% Call up manual drive GUI
h = driveArrows(CreatePort,tagNum);

% declare datastore as a global variable so it can be accessed from the
% workspace even if the program is stopped
global dataStore;

% initialize datalog struct (customize according to needs)
dataStore = struct('truthPose', [],...
                   'odometry', [], ...
                   'rsdepth', [], ...
                   'bump',[],...
                   'beacon',[]);
               
noRobotCount = 0;
tic
while toc<maxTime
    
    % Read and Store Sensore Data
    [noRobotCount,dataStore]=readStoreSensorData(CreatePort,DepthPort,TagPort,tagNum,noRobotCount,dataStore);
    
    % Update Occupancy Grids Here with logOddsBump.m or logOddsDepth.m 
    
    % Plot Occupancy Grid and robot trajectory in real time 
    
    pause(0.1);
end
dataStore.logOddsBump = l;
%dataStore.logOddsDepth = l2;

