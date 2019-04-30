function pointSet = findNeareastPoint(pose,waypoints,kNearest)
robot_xy = [pose(1),pose(2)];
robot_theta=pose(3);
point = [waypoints(1,1),waypoints(1,2)];
[waypointsNum,~] = size(waypoints,1);
if kNearest > waypointsNum
    pointSet = robot_xy;
    disp("ERROR!");
else
    d = findEuclideanDistance(robot_xy,point);
    if kNearest > 1
        for i = 2:waypointsNum
            point = [waypoints(i,1),waypoints(i,2)];
            d = [d;findEuclideanDistance(robot_xy,point)];
        end
    end
end
end


end