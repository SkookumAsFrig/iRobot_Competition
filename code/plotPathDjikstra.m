clc
clear all
close all

load('wptsdebug.mat')
map = 'compMap_mod.mat';
mapstruct = importdata(map);
% mapdata = [mapstruct.map;mapstruct.optWalls];
mapdata = mapstruct.map;
% waypoints = mapstruct.waypoints;
waypoints = wpts_go(:,1:2);

maxX = max([max(mapdata(:,1)) max(mapdata(:,3))]);
maxY = max([max(mapdata(:,2)) max(mapdata(:,4))]);
minX = min([min(mapdata(:,1)) min(mapdata(:,3))]);
minY = min([min(mapdata(:,2)) min(mapdata(:,4))]);
limits = [minX minY maxX maxY];

figure

obstacles = wall2polygon(mapdata,0.27);
obstacles = obstacles(5:end,:);
g = size(obstacles,1);
edges = zeros(g*4,4);
for i=1:g
    for j=1:4
        edges(4*(i-1)+j,:) = obstacles(i,2*(j-1)+1:2*(j-1)+4);
    end
end

axis equal

[m,n] = size(mapdata);
mapext = zeros(m-4,n);
for i=1:m-4
    mapext(i,:) = extendedge(mapdata(i+4,:), 0.26);
end

%start = [-2 1];
start  = [-0.0448   -0.5296];
hold on

[vert, connect_mat] = createRoadmap(obstacles,limits,waypoints,start,mapext,edges);

axis equal

[g,~] = size(vert);
for i=1:g-1
    for j=i+1:g
        if connect_mat(i,j)~=0
            plot([vert(i,1) vert(j,1)],[vert(i,2) vert(j,2)],...
                '--','LineWidth',1,'Color',[0.8500    0.3300    0.1000])
        end
    end
end

[cost,path] = findPath(obstacles,limits,waypoints,start,mapdata,0.26);

plot(path(:,1),path(:,2),'mo-','LineWidth',2,'MarkerFaceColor',[1 0 1])

plot(start(1),start(2),'ko','MarkerFaceColor',[1 0 0])

title('Shortest Path Plot')
xlabel('Global X')
ylabel('Global Y')

