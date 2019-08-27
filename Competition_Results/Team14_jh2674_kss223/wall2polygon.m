function obstacles = wall2polygon(map,radius)
% 
%   Function: obstacles = wall2polygon(map,radius)
%   Input: 
%           map         n x 4 matrix consisting of walls [x1,y1,x2,y2]
%           radius      half of the wall's thickness
%
%   Output:
%           obstacles   n x 10 matrix consisting of all vertices of polygons
%
%   Final Competition
%   Kowin Shi, Jie Huang

[size_map,~] = size(map);
obstacles = [];
for i = 1: size_map
    iniPt = [map(i,1),map(i,2)];
    endPt = [map(i,3),map(i,4)];
    new1 = (endPt - iniPt)/norm(endPt - iniPt)*radius + endPt;
    new2 = (iniPt - endPt)/norm(endPt - iniPt)*radius + iniPt;
    vec_ver1 = cross([0,0,1],[(endPt - iniPt)/norm(endPt - iniPt),0]);
    vec_ver2 = cross([0,0,1],[(iniPt - endPt)/norm(endPt - iniPt),0]);
    e1 = new1+vec_ver1(1:2)*radius;  e2 = new1-vec_ver1(1:2)*radius;
    e3 = new2+vec_ver2(1:2)*radius;  e4 = new2-vec_ver2(1:2)*radius;
    polygon = [e1,e2,e3,e4,e1];
    obstacles = [obstacles;polygon];
    
    %% PLOT
    plot([map(i,1),map(i,3)],[map(i,2),map(i,4)],'k')
    hold on
    for j = 1:4
        plot(obstacles(i,1:2:end),obstacles(i,2:2:end),'b');
    end
end

end

