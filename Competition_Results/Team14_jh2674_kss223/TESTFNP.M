kNearest = 3;
pose = [-1,0.5,0];
goalp = [2.33 0.82;
         2.16 -1.03;
        -2.43 -0.5];
pointSet = findNeareastPoints(pose,goalp,kNearest);