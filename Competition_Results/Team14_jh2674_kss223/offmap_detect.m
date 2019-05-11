function wt = offmap_detect(Mt,map)
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

wt = Mt(:,end);
ind = [];
for i=1:length(wt)
    if Mt(i,1) > maxX || Mt(i,1) < minX || Mt(i,2) > maxY || Mt(i,2) < minY
        wt(i) = 0;
        ind = [ind;i];
    end
end

if sum(wt)==0
    newN = length(wt)-length(ind);
    if newN~=0
        for i=1:length(wt)
            if ~ismember(i,ind)
                wt(i) = 1/newN;
            end
        end
    else
        wt = -1*ones(length(wt),1);
    end
end

end
