function twinPts = findOptWallTwinPts(optWalls,radius)
[wallNum,~] = size(optWalls);
twinPts = [];
for i = 1:wallNum
    pt1 = [optWalls(i,1),optWalls(i,2)];
    pt2 = [optWalls(i,3),optWalls(i,4)];
    center = [1/2 * (pt1(1)+pt2(1)), 1/2 * (pt1(2)+pt2(2))];
    if pt1(1)==pt2(1) && pt1(2)~=pt2(2) % |
        twin1 = [center(1)+radius,center(2)];
        twin2 = [center(1)-radius,center(2)];
    elseif pt1(2)==pt2(2) && pt1(1)~=pt2(1) % -
        twin1 = [center(1),center(2)+radius];
        twin2 = [center(1),center(2)-radius];
    else
        k = (pt1(2)-pt2(2))/(pt1(1)-pt2(1));
        dx = abs(radius*cos(atan(-1/k)));
        dy = abs(radius*sin(atan(-1/k)));
        if k > 0
            twin1 = [center(1)+dx,center(2)-dy];
            twin2 = [center(1)-dx,center(2)+dy];
        else
            twin1 = [center(1)+dx,center(2)+dy];
            twin2 = [center(1)-dx,center(2)-dy];
        end
    end
    twinPts = [twinPts;twin1;twin2];
%     hold on;
%     line([pt1(1),pt2(1)],[pt1(2),pt2(2)]);
%     plot(twin1(1),twin1(2),'rp');
%     plot(twin2(1),twin2(2),'bp');
%     plot(center(1),center(2),'m*')
%     axis equal
end
end