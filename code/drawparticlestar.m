function [a, b, c, d] = drawparticlestar(x,y,theta)
% RANGEPREDICT: predict the range measurements for a robot operating
% in a known map.
%
%   RANGE = RANGEPREDICT(ROBOTPOSE,MAP,ROBOTRAD,ANGLES) returns
%   the expected range measurements for a robot operating in a known
%   map.
%
%   INPUTS
%       robotPose   	3-by-1 pose vector in global coordinates (x,y,theta)
%       map         	N-by-4 matrix containing the coordinates of walls in
%                   	the environment: [x1, y1, x2, y2]
%       sensorOrigin	origin of the sensor-fixed frame in the robot frame: [x y]
%       angles      	K-by-1 vector of the angular orientation of the range
%                   	sensor(s) in the sensor-fixed frame, where 0 points
%                   	forward. All sensors are located at the origin of
%                   	the sensor-fixed frame.
%
%   OUTPUTS
%       depth       	K-by-1 vector of depths (meters)
%
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 4
%   SHI, KOWIN

a = plot(x,y,'kp','MarkerSize',15,'MarkerFaceColor','k');
drawnow;
ptrlength = 0.5;
FOVlength = 5;
angFOV = 27*pi/180;
endptx = x+ptrlength*cos(theta);
endpty = y+ptrlength*sin(theta);
b = plot([x endptx],[y endpty],'k','LineWidth',2);
endptx = x+FOVlength*cos(theta+angFOV);
endpty = y+FOVlength*sin(theta+angFOV);
c = plot([x endptx],[y endpty],'b','LineWidth',1);
endptx = x+FOVlength*cos(theta-angFOV);
endpty = y+FOVlength*sin(theta-angFOV);
d = plot([x endptx],[y endpty],'b','LineWidth',1);

end
