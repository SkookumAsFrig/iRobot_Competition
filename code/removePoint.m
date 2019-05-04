function waypoints = removePoint(pose,waypoints)
%   Function waypoints = removePoint(pose,waypoints)
%   INPUT:
%           pose        1 x 3 or 3 x 1  robot current pose
%           waypoints   n x 3           a set of coordinates of waypoints
%
%   OUTPUT:
%           waypoints   (n-1) x 3       n-1 points
%

    kNearest = 1;
    [neareastPoint,type] = findNeareastPoints(pose,waypoints,kNearest);
    wptsNum = size(waypoints,1);
    
    opt = 0;
    for i = 1:wptsNum
       if neareastPoint(1) == waypoints(i,1) && neareastPoint(2) == waypoints(i,2)
           if waypoints(i,3) == 1
               removeID = i;
               opt = 0;
               disp("remove waypoint");
           elseif waypoints(i,3) == 2
               removeID = i;
               opt = 0;
               disp("remove EC waypoint");
           else
               removeID = i;
               opt = 1;
               disp("remove twin points");
           end
       end
    end
    if opt == 0
        waypoints(removeID,:) = [];
    else
        if mod(waypoints(removeID,3),2)==1 % odd idx
            waypoints(removeID:removeID+1,:) = [];
        else % even idx
            waypoints(removeID-1:removeID,:) = [];
        end
    end
end