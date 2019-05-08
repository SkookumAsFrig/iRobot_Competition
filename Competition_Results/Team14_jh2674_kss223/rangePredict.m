function[range] = rangePredict(robotPose,map,sensorOrigin,angles)
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
%       range       	K-by-1 vector of ranges (meters)
%
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 2
%   SHI, KOWIN

%get map rows for iteration later
[N,~] = size(map);
%sensor point 2 distance scaling factor, in meters
alph = 100;
%get sensor xy in robot frame
sensorx_rframe = sensorOrigin(1);
sensory_rframe = sensorOrigin(2);
%get pose in global frame
robotx = robotPose(1);
roboty = robotPose(2);
robottheta = robotPose(3);
%get transformation matrix from robot to global
tvec = [cos(robottheta) -sin(robottheta) robotx;...
    sin(robottheta) cos(robottheta) roboty; 0 0 1];
out = zeros(3,length(angles));
%calculate the end point (point 2) of sensor "laser" vector
for i=1:length(angles)
    point2sensorx_rframe = alph*cos(angles(i))+sensorx_rframe;
    point2sensory_rframe = alph*sin(angles(i))+sensory_rframe;
    out(:,i) = tvec*[point2sensorx_rframe point2sensory_rframe 1]';
end

sense = tvec*[sensorx_rframe sensory_rframe 1]';
%beginning points in global frame
sensorx_iframe = sense(1);
sensory_iframe = sense(2);
%end points in global frame
point2sensorx_iframe = out(1,:);
point2sensory_iframe = out(2,:);
%initialize range, note that out of range is sensed as maxscale
maxscale = 10;
range = maxscale*ones(length(angles),1);

for j=1:length(angles)
    %iterate through all angles of sensors
    for k=1:N
        %check each map line segment
        x3= map(k,1);
        y3= map(k,2);
        x4= map(k,3);
        y4= map(k,4);
        [isect,x,y,ua]= intersectPoint(sensorx_iframe,sensory_iframe...
            ,point2sensorx_iframe(j),point2sensory_iframe(j),x3,y3,x4,y4);
        %get intersection point, if any
        if(isect && ua*alph <= range(j))
            %if it exists, then take the minimum distance as the range
            range(j) = ua*alph;
        end
    end
end

end
