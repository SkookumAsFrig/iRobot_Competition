clc
clear all
close all

workspacetxt = 'compMap_big.mat';
mapdata = importdata(workspacetxt);
mapdata = mapdata.map;
[l,~] = size(mapdata);
xmin = min(min([mapdata(:,1) mapdata(:,3)]));
ymin = min(min([mapdata(:,2) mapdata(:,4)]));
xmax = max(max([mapdata(:,1) mapdata(:,3)]));
ymax = max(max([mapdata(:,2) mapdata(:,4)]));
limits = [xmin ymin xmax ymax];

init = [-4 -3];
goal = [3.5 3];
% goal = [1.5 1.5];
% goal = [0 -1];
goal2 = [-1 -3.5];
goal3 = [-4.5 3.5];
start_goal = [init;goal];

figure
hold on
for j=1:l
    a=plot([mapdata(j,1) mapdata(j,3)],[mapdata(j,2) mapdata(j,4)],'LineWidth',2,'Color','k','HandleVisibility','off');
end

axis equal

%sampling_handle = @(limits,lastind) lowdisp_resample(limits,lastind);
sampling_handle = @(limits) uniformresample(limits);
stepsize = 0.2;
radius = 0.25;
numbGoals = 3;
% [newV,newconnect_mat,cost,path,pathpoints] = buildBIRRT(workspacetxt,limits,sampling_handle,init,goal,stepsize,radius);
[newV,newconnect_mat,cost,path,pathpoints,expath,expoint,expath2,expoint2,timeup] ...
    = buildQuadRRT(workspacetxt,limits,sampling_handle,init,goal,goal2,goal3,stepsize,radius,numbGoals);
plot(pathpoints(:,1),pathpoints(:,2),'mo-','LineWidth',2,'MarkerFaceColor',[1 0 1])

plot(newV(:,1),newV(:,2),'k*')

plot(init(1),init(2),'ko','MarkerFaceColor',[1 0 0])
plot(goal(1),goal(2),'ko','MarkerFaceColor',[0 1 0])

title('Workspace and Bi-Directional RRT, Uniform Random')
xlabel('Global X')
ylabel('Global Y')
