clc
clear all
close all

load('goodfinalrun.mat')
map = 'compMap_mod.mat';
mapstruct = importdata(map);
beaconmat = mapstruct.beaconLoc;
[o,~] = size(beaconmat);
mapdata = mapstruct.map;
waypoint = mapstruct.waypoints;
ECwaypoint = mapstruct.ECwaypoints;
[l,~] = size(mapdata);
maxX = max([max(mapdata(:,1)) max(mapdata(:,3))]);
maxY = max([max(mapdata(:,2)) max(mapdata(:,4))]);
minX = min([min(mapdata(:,1)) min(mapdata(:,3))]);
minY = min([min(mapdata(:,2)) min(mapdata(:,4))]);

xrange = (maxX-minX);
yrange = (maxY-minY);

figure
initsw = 0;
numpart = size(dataStore.particles(:,:,1),1);
sensorOrigin = [0.187 0.018];

partTraj = [];

filename = 'testAnimated_fixed.gif';

seeBeacInd = 1;
tempglob = zeros(9,2);

for i=1:420
    clf
    disp(i)
    hold on
    currtime = dataStore.rsdepth(i,1);
    [~,I] = min(abs(dataStore.truthPose(:,1) - currtime));
    for j=1:l
        a=plot([mapdata(j,1) mapdata(j,3)],[mapdata(j,2) mapdata(j,4)],'LineWidth',2,'Color','k','HandleVisibility','off');
    end
    
    d=plot(ECwaypoint(:,1),ECwaypoint(:,2),'k+','MarkerSize',15);
    t=plot(waypoint(:,1),waypoint(:,2),'bs','MarkerSize',15,'MarkerFaceColor','g');
    
    for e=1:o
        b=plot(beaconmat(e,2),beaconmat(e,3),'rp','MarkerFaceColor','r');
        text(beaconmat(e,2),beaconmat(e,3),num2str(beaconmat(e,1)))
    end
    
    for k=1:1:numpart
        c=drawparticle(dataStore.particles(k,1,i),dataStore.particles(k,2,i),dataStore.particles(k,3,i));
    end
    
    xpartmean = mean(dataStore.particles(:,1,i));
    ypartmean = mean(dataStore.particles(:,2,i));
    tpartmean = mean(dataStore.particles(:,3,i));
    partTraj = [partTraj; [xpartmean ypartmean tpartmean]];
    
    f=plot(dataStore.truthPose(1:I,2),dataStore.truthPose(1:I,3),'b');
    g=plot(partTraj(:,1),partTraj(:,2),'r');
    
    h=drawparticlestar_red(xpartmean,ypartmean,tpartmean);
    k=drawparticlestar(dataStore.truthPose(I,2),dataStore.truthPose(I,3),dataStore.truthPose(I,4));
    
    firstnone = find(dataStore.timebeacon(i,:)==-1,1);
    beacnumb = (firstnone-1)/3;
    if beacnumb == 0
        m=plot(20,20,'ro','MarkerFaceColor',[1 .6 .6]);
    else
    for j=1:beacnumb
        beacpoint = robot2global(dataStore.truthPose(i,2:4),dataStore.timebeacon(i,(j-1)*3+2:(j-1)*3+3));
        m=plot(beacpoint(1),beacpoint(2),'ro','MarkerFaceColor',[1 .6 .6]);
    end
    end
    
    if beacnumb == 0
        n=plot(20,20,'ro','MarkerFaceColor',[1 0 .6]);
    else
    for j=1:beacnumb
        beacpoint = robot2global(partTraj(i,:),dataStore.timebeacon(i,(j-1)*3+2:(j-1)*3+3));
        n=plot(beacpoint(1),beacpoint(2),'ro','MarkerFaceColor',[1 0 .6]);
    end
    end
    
    
    depthR = dataStore.rsdepth(i,3:end);
    depthXY = depth2xy(depthR, sensorOrigin);
    for j=1:9
        tempglob(j,:) = robot2global(dataStore.truthPose(I,2:4),[depthXY(1,j) depthXY(2,j)]);
    end
    y=plot(tempglob(:,1),tempglob(:,2),'*');
    
    for j=1:9
        tempglob(j,:) = robot2global(partTraj(i,:),[depthXY(1,j) depthXY(2,j)]);
    end
    p=plot(tempglob(:,1),tempglob(:,2),'*');
    
    x0=300;
    y0=300;
    width=720;
    height=600;
    set(gcf,'position',[x0,y0,width,height])
    xlim([-3, 3]);
    ylim([-3.5, 3.5]);
    
    xlabel('Global X (m)')
    ylabel('Global Y (m)')
    
    legend([a b c d t f g h k m n y p],'Map','Beacons','Particles','EC Waypoints',...
        'Waypoints','truthPose Trajectory','Robot Estimate Trajectory',...
        'Robot Estimate','truthPose','truthPose View of Beacon','Robot Estimate of Beacon',...
        'truthPose Depth','Robot Estimate Depth','Location','northeastoutside')
    
    
    title(['Time is ',num2str(i)])
    ax = gca;
    ax.Units = 'pixels';
    pos = ax.Position;
    ti = ax.TightInset;
    rect = [-ti(1), -ti(2), 1.5*(pos(3)+ti(1)+ti(3)), pos(4)+ti(2)+ti(4)];
    F = getframe(ax,rect);
    [A,map] = rgb2ind(frame2im(F),256);
    
    if i == 1
        imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',0.3);
    else
        imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',0.3);
    end
end