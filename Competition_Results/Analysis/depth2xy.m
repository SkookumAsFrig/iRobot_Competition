function[depthXY] = depth2xy(depthR, offset)
% LIDAR_RANGE2XY: convert lidar measurements (range & bearing) into x/y
% coordinates (in robot local frame)
%
%   LIDARXY = LIDAR_RANGE2XY(LIDARR,ROBOTRAD,ANGRANGE,N) returns the
%   x/y coordinates (in robot local frame) of lidar measurements.
%
%   INPUTS
%       depthR      1-by-9 vector of scan ranges (meters)
%       offset      camera offset in robot frame 1x2 (meters)
%
%   OUTPUTS
%       lidarXY     2-by-9 matrix of x/y scan locations
%
%   NOTE: Assume lidar is located at front of robot and pointing forward
%         (along robot's x-axis).
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   SHI, KOWIN

depthXY = zeros(2,9); %initialization
angRange = 27*2*pi/180;
anghalf = angRange/2;
anglevec = linspace(anghalf,-anghalf,9); %equally spaced angles
for k = 1:9
    depthXY(1,k) = offset(1) + depthR(k); %depth to x
    depthXY(2,k) = offset(2) + depthR(k)*tan(anglevec(k)); %depth to y
end
end
