function[xyR] = global2robot(pose,xyG)
% GLOBAL2ROBOT: transform a 2D point in global coordinates into robot
% coordinates (assumes planar world).
% 
%   XYR = GLOBAL2ROBOT(POSE,XYG) returns the 2D point in robot coordinates
%   corresponding to a 2D point in global coordinates.
% 
%   INPUTS
%       pose    robot's current pose [x y theta]  (1-by-3)
%       xyG     2D point in global coordinates (1-by-2)
% 
%   OUTPUTS
%       xyR     2D point in robot coordinates (1-by-2)
% 
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   SHI, KOWIN

x = pose(1);
y = pose(2);
theta = pose(3);
%transformation matrix global to local
Tpose = [cos(theta) sin(theta);-sin(theta) cos(theta)]*[-x; -y];
xr = Tpose(1);
yr = Tpose(2);
%use these for the full transformation matrix
TBI = [cos(theta) sin(theta) xr;-sin(theta) cos(theta) yr; 0 0 1];
%augment state
wiaug = [xyG'; 1];
xyRout = TBI*wiaug;
%select output state
xyR = xyRout(1:2)';

end