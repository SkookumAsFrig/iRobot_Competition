clc
clear all
load('beaconPredicttestData2.mat')
%gg = {robotPose,sensorOrigin,map,n,beaconmat,m};
robotPose = gg{1};
sensorOrigin = gg{2};
map = gg{3};
n = gg{4};
beaconmat = gg{5};
m = gg{6};

[canSee, coordinates] = beaconPredict(robotPose,sensorOrigin,map,n,beaconmat,m);

figure
axis equal
hold on
for j=1:n
    plot([map(j,1) map(j,3)],[map(j,2) map(j,4)],'LineWidth',2,'Color','k','HandleVisibility','off')
end

for e=1:m
    plot(beaconmat(e,2),beaconmat(e,3),'rp','MarkerFaceColor','r')
end

for e=1:m
    plot(beaconmat(e,2),beaconmat(e,3),'rp','MarkerFaceColor','r')
    text(beaconmat(e,2),beaconmat(e,3),num2str(beaconmat(e,1)))
end

[g, j] = drawparticlestar(robotPose(1),robotPose(2),robotPose(3));
% origin = robot2global([robotPose(1) robotPose(2) robotPose(3)],sensorOrigin);
% onepoint = robot2global([origin robotPose(3)],[4 -4]);
% otherpoint = robot2global([origin robotPose(3)],[4 4]);
%
% matforplot = [onepoint; origin; otherpoint];
% plot(matforplot(:,1),matforplot(:,2));

beaconmat(logical(canSee),:)
