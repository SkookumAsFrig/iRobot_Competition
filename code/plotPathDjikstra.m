clc
clear all
close all

map = 'compMap_nobeacon.mat';
mapstruct = importdata(map);
mapdata = mapstruct.map;
limits = [0 0 100 100];

figure

obstacles = wall2polygon(mapdata,0.25);

initialp = [80 95];
goalp = [10 50];
start_goal = [initialp;goalp];
[vert, connect_mat] = createRoadmap(obstacles,limits,start_goal);

plot([limits(1) limits(1) limits(3) limits(3) limits(1)],...
    [limits(2) limits(4) limits(4) limits(2) limits(2)],'k--','LineWidth',2)
hold on

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

[cost,path,vertices] = findPath(obstacles,limits,initialp,goalp);

plot(vertices(path,1),vertices(path,2),'mo-','LineWidth',2,'MarkerFaceColor',[1 0 1])

plot(initialp(1),initialp(2),'ko','MarkerFaceColor',[1 0 0])
plot(goalp(1),goalp(2),'ko','MarkerFaceColor',[0 1 0])

title('Shortest Path Plot, Environment = hw8.txt')
xlabel('Global X')
ylabel('Global Y')

