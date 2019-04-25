function [noRobotCount,dataStore]=readStoreSensorData(CreatePort,DistPort,TagPort,tagNum,noRobotCount,dataStore)
% This function tries to read all the sensor information from the Create
% and store it in a data structure
%
%   [noRobotCount,dataStore]=readStoreSensorData(CreatePort,DepthPort,TagPort,tagNum,noRobotCount,dataStore) runs 
% 
%   INPUTS
%       CreatePort   Create ports object (get from running CreatePiInit)
%       DistPort    Depth ports object (get from running CreatePiInit)
%       TagPort      Tag ports object (get from running CreatePiInit)
%       tagNum       robot number for overhead localization
%       noRobotCount Number of consecutive times the robot was "lost" by the overhead localization
%       dataStore    struct containing logged data
% 
%   OUTPUTS
%       noRobotCount   Updated number of consecutive times the robot was "lost" by the overhead localization
%       dataStore   Updated struct containing logged data
%
% 
%   Cornell University
%   MAE 5180: Autonomous Mobile Robots


    % read truth pose (from overhead localization system)
    try
        [px, py, pt] = OverheadLocalizationCreate(tagNum);
        if (px == 0 && py == 0 && pt == 0 && ~isa(tagNum,'CreateRobot'))
            disp('Overhead localization lost the robot!')
            noRobotCount = noRobotCount + 1;
        else
            poseX = px; poseY = py; poseTheta = pt;
            dataStore.truthPose = [dataStore.truthPose ; ...
                               toc poseX poseY poseTheta];
            noRobotCount = 0;
        end
    catch
        disp('Error retrieving or saving overhead localization data.');
    end
%     
%     %read odometry distance & angle
%     try
%         deltaD = DistanceSensorRoomba(CreatePort);
%         deltaA = AngleSensorRoomba(CreatePort);
%         dataStore.odometry = [dataStore.odometry ; ...
%                               toc deltaD deltaA];
%     catch
%         disp('Error retrieving or saving odometry data.');
%     end
%     
%  
%     
%     % read bump data
%     try
%         [BumpRight BumpLeft DropRight DropLeft DropCaster BumpFront] = ...
%             BumpsWheelDropsSensorsRoomba(CreatePort);
%         dataStore.bump = [dataStore.bump ; toc ...
%             BumpRight BumpLeft DropRight DropLeft DropCaster BumpFront];
%     catch
%         disp('Error retrieving or saving bump sensor data.');
%     end
%     
% 
% 
%     %read Real Sense depth data
%     try
%            
%         depth_array = RealSenseDist(DistPort);
%         dataStore.rsdepth = [dataStore.rsdepth ; toc depth_array'];
%     catch
%         disp('Error retrieving or saving RealSense depth data.');
%     end
% 
%     %read camera data (beacons)
%     try
%         tags = RealSenseTag(TagPort);
%         if ~isempty(tags)
%             dataStore.beacon = [dataStore.beacon ; repmat(toc,size(tags,1),1) tags];
%         end
%     catch
%         disp('Error retrieving or saving beacon (AprilTag) data.');
%     end
