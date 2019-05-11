function Gt = GjacDiffDrive(mubar, d, phi)
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

p = phi;
if p~=0
    %p=0 means robot did not turn, but in this case it did
    R = d/(2*p);
    deltax = 2*R*sin(p)*sin(pi/2-p);
    deltay = -2*R*sin(p)*cos(pi/2-p);
    %implementation of 1.1 algorithm
else
    %robot did not turn, only move in local x axis
    deltax = d;
    deltay = 0;
end

Gt = [1 0 -deltax*sin(thetatbar)-deltay*cos(thetatbar);...
    0 1 deltax*cos(thetatbar)-deltay*sin(thetatbar);0 0 1];

end
