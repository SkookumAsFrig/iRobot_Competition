function[xt,yt,thetat] = integrateOdom(xi, yi, thetai, dvec, phivec)
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

% initialization
xt = zeros(1,length(dvec)+1);
yt = zeros(1,length(dvec)+1);
thetat = zeros(1,length(dvec)+1);

xt(1) = xi;
yt(1) = yi;
thetat(1) = thetai;
% computation
for i=1:length(dvec)
    %compute next timestep incrementally
    d = dvec(i);
    p = phivec(i);
    %get current d and phi values
    if p~=0
        %p=0 means robot did not turn, but in this case it did
        R = d/p;
        deltax = 2*R*sin(p/2)*sin(pi/2-p/2);
        deltay = -2*R*sin(p/2)*cos(pi/2-p/2);
        %implementation of 1.1 algorithm
    else
        %robot did not turn, only move in local x axis
        deltax = d;
        deltay = 0;
    end
    %all calculation done above is in local coordinates, transfrom to
    %global using previous pose
    tvec = [cos(thetat(i)) -sin(thetat(i)) xt(i);...
        sin(thetat(i)) cos(thetat(i)) yt(i); 0 0 1]*[deltax;deltay;1];
    %get next configuration, which is used as the last configuration for
    %the next iteration
    xt(i+1) = tvec(1);
    yt(i+1) = tvec(2);
    thetat(i+1) = thetat(i)+p;
end

end
