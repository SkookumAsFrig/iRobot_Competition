function Mt = resample(map,numpart)
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

maxX = max([max(map(:,1)) max(map(:,3))]);
maxY = max([max(map(:,2)) max(map(:,4))]);
minX = min([min(map(:,1)) min(map(:,3))]);
minY = min([min(map(:,2)) min(map(:,4))]);

xhalf = (maxX-minX)/2;
yhalf = (maxY-minY)/2;

xpart = xhalf*rand(numpart,1)+minX;
ypart = yhalf*rand(numpart,1)+minY;
thepart = 2*pi*rand(numpart,1);
wi = ones(numpart,1)/numpart;
Mt = [xpart ypart thepart wi];

end
