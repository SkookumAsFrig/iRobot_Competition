function[vout,wout,reached] = visitWaypoints(waypoints,gotopt,closeEnough,epsilon, alph, x, y, theta)
% visitWaypoints: calculates the desired [V w] controls to drive the robot 
% along a series of waypoints using feedbackLin.m. The waypoints are given 
% as a nx2 matrix where each row is the (x,y) coordinate of a waypoint. 
% Keeps track of which waypoint is being driven toward using the index 
% gotopt, and considers the waypoint reached if the robot is within 
% closeEnough of the waypoint location.
%
%   [vout,wout,reached] = visitWaypoints(waypoints,gotopt,closeEnough,
%   epsilon, alph, x, y, theta) returns the forward and angular velocity
%   commands to drive a differential-drive robot, and signals reached when
%   closeEnough condition is satisfied with current waypoint.
%
%   INPUTS
%       waypoints   nx2 matrix, each row is [x y] in meters in global frame
%       gotopt      row index of current waypoint
%       closeEnough distance to waypoint that robot can consider it reached
%       epsilon     feedback linearization constant
%       alph        velocity vector scaling factor
%       x           robot global pose
%       y           robot global pose
%       theta       robot angular pose
%
%   OUTPUTS
%       vout        forward velocity command (m/s)
%       wout        angular velocity command (rad/s)
%       reached     binary condition for being closeEnough to waypoint
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   SHI, KOWIN

pointnow = waypoints(gotopt,:);
%obtain current point

dist = pointnow - [x y];
distx = dist(1);
disty = dist(2);

Vee = alph*[distx,disty];
Vx = Vee(1);
Vy = Vee(2);
%calculate control input with scaling factor

[vout,wout] = feedbackLin(Vx,Vy,theta,epsilon);
%use feedback linearization appropriately

%signals reached condition
if norm(dist)<=closeEnough
    reached = 1;
else
    reached = 0;
end

end
