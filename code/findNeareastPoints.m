function pointSet = findNeareastPoints(pose,waypoints,kNearest)
%   Function pointSet = findNeareastPoints(pose,waypoints,kNearest)
%   INPUT:
%           pose        1 x 3 or 3 x 1 robot current pose
%           waypoints   n x 2 a set of coordinates of waypoints
%           kNearest    the number of first k neareast points
%           updateMap   m x 4 a matrix contain updated map
%
%   OUTPUT:
%           pointSet    k x 2
%
robot_xy = [pose(1),pose(2)];
robot_theta=pose(3);
point = [waypoints(1,1),waypoints(1,2)];
[waypointsNum,~] = size(waypoints);
cross = [];
wallID = [];
if kNearest > waypointsNum
    pointSet = robot_xy;
    disp("ERROR_1: k value is larger than waypoints number");
elseif kNearest == waypointsNum
    pointSet = waypoints;
else
    d = findEuclideanDistance(robot_xy,point);
    if kNearest > 1
        for i = 2:waypointsNum
            point = [waypoints(i,1),waypoints(i,2)];
            d = [d; findEuclideanDistance(robot_xy,point)];
        end
        if ~isempty(d)
            [serial,idx]=sort(d);
            kIdx = idx(1:kNearest);
            pointSet = waypoints(kIdx,:);
        else
            pointSet = robot_xy+robot_xy+robot_xy;
            disp("ERROR_3: No available waypoint!");
        end
    else
        pointSet = robot_xy+robot_xy;
        disp("ERROR_2: k value is smaller than 1!");
    end
end
end
