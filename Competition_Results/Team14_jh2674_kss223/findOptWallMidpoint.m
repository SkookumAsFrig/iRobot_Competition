function midpoints = findOptWallMidpoint(optWalls)
[wallNum,~] = size(optWalls);
midpoints = [];
for i = 1:wallNum
    pt1 = [optWalls(i,1),optWalls(i,2)];
    pt2 = [optWalls(i,3),optWalls(i,4)];
    center = [1/2 * (pt1(1)+pt2(1)), 1/2 * (pt1(2)+pt2(2))];
    midpoints = [midpoints;center];
    %plot(center(1),center(2),'rp');
    %hold on;
end
end