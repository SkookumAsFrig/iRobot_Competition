function waypoints = removePoint(pose,waypoints)
    kNearest = 1;
    neareastPoint = findNeareastPoints(pose,waypoints,kNearest);
    wptsNum = size(waypoints,1);
    for i = 1:wptsNum
       if neareastPoint(1) == waypoints(i,1) && neareastPoint(2) == waypoints(i,2)
           if waypoints(i,3) == 1
               removeID = i;
               disp("remove waypoint");
           elseif waypoints(i,3) == 2
               removeID = i;
               disp("remove EC waypoint");
           end
       end
    end
    waypoints(removeID,:) = [];
end