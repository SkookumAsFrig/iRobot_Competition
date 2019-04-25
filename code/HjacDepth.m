function [Hdepth] = HjacDepth(mubarbar, mubar, map)
% hGPS: predict the depth measurements for a robot operating
% in a known map, given the expected range measurements.
%
%   depth = depthPredict(angles, range) returns
%   the expected depth measurements for a robot given range measurements.
%
%   INPUTS
%       angles      	K-by-1 vector of the angular positions of the range
%                   	sensor(s) in robot coordinates, where 0 points forward
%       range           K-by-1 vector of ranges (meters)
%
%   OUTPUTS
%       depth       	K-by-1 vector of depths (meters)
%
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 4
%   SHI, KOWIN

xtbar = mubar(1);
ytbar = mubar(2);
thetatbar = mubar(3);

angles = linspace(-27*pi/180,27*pi/180,9);
sensorOrigin = [0.13 0];
robotPose  = mubar;
range = rangePredict(robotPose,map,sensorOrigin,angles);
depth = depthPredict(angles, range);

robotPosebar  = mubarbar;
rangebar = rangePredict(robotPosebar,map,sensorOrigin,angles);
depthbar = depthPredict(angles, rangebar);

deltah = depth-depthbar;
deltax = mubarbar(1) - xtbar;
deltay = mubarbar(2) - ytbar;
deltatheta = mubarbar(3) - thetatbar;

if deltax == 0
    deltax = 0.0001;
end

if deltay == 0
    deltay = 0.0001;
end

if deltatheta == 0
    deltatheta = 0.0001;
end

Hdepth = [deltah./deltax deltah./deltay deltah./deltatheta];

end
