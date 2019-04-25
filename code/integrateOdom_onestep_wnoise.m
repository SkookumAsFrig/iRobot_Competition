function newstate = integrateOdom_onestep_wnoise(xi, yi, thetai, d, phi)
% integrateOdom: Dead reckoning integration of the robot odometry from
% travel distance and angle data, given initial pose.
%
%   [xt,yt,thetat] = integrateOdom(xi, yi, thetai, dvec, phivec) returns
%   the trajectory of the robot's configuration over time, for x, y and
%   theta.
%
%   INPUTS
%       xi      initial configuration (t=1)                  x
%       yi      initial configuration (t=1)                  y
%       thetai  initial configuration (t=1)                  theta
%       dvec    incremental distance vector (begins at t=2)  1xN [m]
%       phivec  incremental angle vector (begins at t=2)     1xN [rad]
%
%   OUTPUTS
%       xt          1-by-N+1 matrix of x trajectory
%       yt          1-by-N+1 matrix of y trajectory
%       thetat      1-by-N+1 matrix of theta trajectory
%
%   NOTE: Assume differential-drive robot whose wheels turn at a constant
%         rate between sensor readings.
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #2

noise = normrnd(0,sqrt(0.01),3,1);

% computation
%compute next timestep
if phi~=0
    %p=0 means robot did not turn, but in this case it did
    R = d/phi;
    deltax = 2*R*sin(phi/2)*sin(pi/2-phi/2);
    deltay = -2*R*sin(phi/2)*cos(pi/2-phi/2);
    %implementation of 1.1 algorithm
else
    %robot did not turn, only move in local x axis
    deltax = d;
    deltay = 0;
end
%all calculation done above is in local coordinates, transfrom to
%global using previous pose
tvec = [cos(thetai) -sin(thetai) xi;...
    sin(thetai) cos(thetai) yi; 0 0 1]*[deltax;deltay;1];
%get next configuration, which is used as the last configuration for
%the next iteration
xt = tvec(1);
yt = tvec(2);
thetat = thetai+phi;
newstate = zeros(3,1);
newstate(1) = xt;
newstate(2) = yt;
newstate(3) = thetat;
newstate = newstate + noise;

end
