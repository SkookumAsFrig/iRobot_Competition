function [pointSet,type] = findNeareastPoints(pose,waypoints,kNearest)
%   Function pointSet = findNeareastPoints(pose,waypoints,kNearest)
%   INPUT:
%           pose        1 x 3 or 3 x 1 robot current pose
%           waypoints   n x 3 a set of coordinates of waypoints
%           kNearest    the number of first k neareast points
%           updateMap   m x 4 a matrix contain updated map
%
%   OUTPUT:
%           pointSet    k x 2   k nearest points
%           type        k x 1   signals indicates the type of the point
%
robot_xy = [pose(1),pose(2)];
firstPt = [waypoints(1,1),waypoints(1,2)];
[waypointsNum,~] = size(waypoints);
pointSet = [];
type = [];
if kNearest > waypointsNum
    pointSet = robot_xy;
    type = 0;
    disp("ERROR_1: k value is larger than waypoints number");
elseif kNearest == waypointsNum
    pointSet = waypoints(:,1:2);
    type = waypoints(:,3);
else
    d = findEuclideanDistance(robot_xy,firstPt);
    if kNearest > 1
        for i = 2:waypointsNum
            point = [waypoints(i,1),waypoints(i,2)];
            d = [d; findEuclideanDistance(robot_xy,point)];
        end
        [serial,idx]=sort(d);
        kIdx = idx(1:kNearest);
        pointSet = waypoints(kIdx,1:2);
        type = waypoints(kIdx,3);
    elseif kNearest == 1
        for i = 2:waypointsNum
            point = [waypoints(i,1),waypoints(i,2)];
            temp = findEuclideanDistance(robot_xy,point);
            if temp < d
               d = temp;
               pointSet = point;
               type = waypoints(i,3);
            else
                if i == waypointsNum && isempty(pointSet) && isempty(type)
                    pointSet = firstPt;
                    type = waypoints(1,3);                    
                end
            end
        end
    else
        pointSet = robot_xy+robot_xy;
        type = -1;
        disp("ERROR_2: k value is smaller than 1!");
    end
end
end
