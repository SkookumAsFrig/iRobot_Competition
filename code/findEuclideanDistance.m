function distance = findEuclideanDistance(startpt,endpt)
    pt1 = [startpt(1),startpt(2)];
    pt2 = [endpt(1),endpt(2)];
    distance = sqrt((pt1(1)-pt2(1))^2+(pt1(2)-pt2(2))^2);
end