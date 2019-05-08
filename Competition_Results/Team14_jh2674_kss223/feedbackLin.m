function[vout,wout] = feedbackLin(Vx,Vy,theta,epsilon)
% feedbackLin: transform Vx and Vy commands into corresponding V and w
% commands using the feedback linearization technique presented in class 
% for a differential drive robot.
%
%   [vout,wout] = feedbackLin(Vx,Vy,theta,epsilon) returns the
%   forward and angular velocity commands to drive a differential-drive
%   robot
%
%   INPUTS
%       Vx      desired x velocity vector (m/s)
%       Vy      desired y velocity vector (m/s)
%       theta   robot angular pose
%       epsilon feedback linearization constant
%
%   OUTPUTS
%       vout        forward velocity command (m/s)
%       wout        angular velocity command (rad/s)
%
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   SHI, KOWIN

epvec = [1 0;0 1/epsilon];
Rbi = [cos(theta) sin(theta);-sin(theta) cos(theta)];
outvec = epvec*Rbi*[Vx;Vy];
%matrix operation as shown in lecture
vout = outvec(1);
wout = outvec(2);

end
