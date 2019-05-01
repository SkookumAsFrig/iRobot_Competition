function weight = findWeight_comp(robotPose,map,sensorOrigin,angles,zt,beaconmat,DRweight)
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

%[g, j] = drawparticlestar_green(robotPose(1),robotPose(2),robotPose(3));

[canSee, coordinates] = beaconPredict(robotPose,sensorOrigin,map,n,beaconmat,m);

ldp = length(depth);
beacondata = zt(ldp+1:end-3);
deadreck = zt(end-2:end);
wt = zeros(ldp,1);
wtbeacon = zeros(m,1);
wtDR = ones(3,1);
for i=1:ldp
    wt(i) = normpdf(zt(i),depth(i),sqrt(0.5));
end

if DRweight ~= 0
    wtDR(1) = normpdf(robotPose(1),deadreck(1),sqrt(0.1));
    wtDR(2) = normpdf(robotPose(2),deadreck(2),sqrt(0.1));
    wtDR(3) = normpdf(robotPose(3),deadreck(3),sqrt(0.1));
    wtDR = DRweight*wtDR;
end

highprob = 1;
lowprob = 0.00000001;
beacondistW = sqrt(0.01);
bclabels = beacondata(1:3:end);

for i=1:m
    [isin,ind] = ismember(beaconmat(i,1),bclabels);
    if isin && canSee(i)==1%can see current beacon and beacon should be able to be seen
        bc_coord = beacondata((ind-1)*3+2:(ind-1)*3+3);
        wtbeacon(i) = 100*normpdf(bc_coord(1),coordinates(i,1),beacondistW)*normpdf(bc_coord(2),coordinates(i,2),beacondistW);
        %disp(wtbeacon(i))
    elseif (isin && canSee(i)==0) || (~isin && canSee(i)==1)
        wtbeacon(i) = lowprob;
    else
        wtbeacon(i) = highprob;
    end
end

weight = prod([wt; wtbeacon; wtDR]);

% delete(g)
% delete(j)

end
