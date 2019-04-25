function weight = findWeight(robotPose,map,sensorOrigin,angles,zt,beaconmat)
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
[m,~] = size(beaconmat);
[n,~] = size(map);
range = rangePredict(robotPose,map,sensorOrigin,angles);
depth = depthPredict(angles, range);
[canSee, coordinates] = beaconPredict(robotPose,sensorOrigin,map,n,beaconmat,m);
beacondata = zt(end-2:end);
wt = zeros(length(depth),1);
wtbeacon = zeros(m,1);
for i=1:length(depth)
    wt(i) = normpdf(zt(i),depth(i),sqrt(0.1));
end

lowprob = 0.000001;
highprob = 1;

for i=1:m
    if beacondata(1)==beaconmat(i,1) && canSee(i)==1%can see current beacon and beacon should be able to be seen
        wtbeacon(i) = 10*normpdf(beaconmat(i,2),coordinates(i,1),sqrt(2))*normpdf(beaconmat(i,3),coordinates(i,2),sqrt(2));
        disp(wtbeacon(i))
    elseif (beacondata(1)==beaconmat(i,1) && canSee(i)==0) || (beacondata(1)~=beaconmat(i,1) && canSee(i)==1)
        wtbeacon(i) = lowprob;
    else
        wtbeacon(i) = highprob;
    end
end


weight = prod([wt; wtbeacon]);

end
