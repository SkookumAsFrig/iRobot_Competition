function[depth] = depthPredict(angles, range)
% depthPredict: predict the depth measurements for a robot operating
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
%   Homework 2
%   SHI, KOWIN

%initialize output
depth = zeros(length(angles),1);
%sort to match RealSenseDist output convention, positive-most angle first
[anglessorted,indsorted] = sort(angles,'descend');
for i=1:length(angles)
    depth(i,1) = range(indsorted(i))*cos(anglessorted(i));
    %get depth by trignometery with range data
end

end
