clc
clear all
close all

load('bigmapdebug.mat')
map = 'compMap.mat';
mapstruct = importdata(map);
beaconmat = mapstruct.beaconLoc;
[o,~] = size(beaconmat);
mapdata = mapstruct.map;
[l,~] = size(mapdata);
maxX = max([max(mapdata(:,1)) max(mapdata(:,3))]);
maxY = max([max(mapdata(:,2)) max(mapdata(:,4))]);
minX = min([min(mapdata(:,1)) min(mapdata(:,3))]);
minY = min([min(mapdata(:,2)) min(mapdata(:,4))]);

xrange = (maxX-minX);
yrange = (maxY-minY);

figure
initsw = 0;
numpart = size(dataStore.particles(:,:,1),1)
sensorOrigin = [0.15 0.06];

partTraj = [];

filename = 'testAnimated_fixed.gif';

seeBeacInd = 1;
tempglob = zeros(9,2);

for i=1:length(dataStore.truthPose(:,1))
    clf
    disp(i)
    hold on
    
    for j=1:l
        plot([mapdata(j,1) mapdata(j,3)],[mapdata(j,2) mapdata(j,4)],'LineWidth',2,'Color','k','HandleVisibility','off')
    end
    for e=1:o
        plot(beaconmat(e,2),beaconmat(e,3),'rp','MarkerFaceColor','r')
        text(beaconmat(e,2),beaconmat(e,3),num2str(beaconmat(e,1)))
    end
    
    for k=1:1:numpart
        drawparticle(dataStore.particles(k,1,i),dataStore.particles(k,2,i),dataStore.particles(k,3,i));
    end
    
    xpartmean = mean(dataStore.particles(:,1,i));
    ypartmean = mean(dataStore.particles(:,2,i));
    tpartmean = mean(dataStore.particles(:,3,i));
    partTraj = [partTraj; [xpartmean ypartmean tpartmean]];
    
    plot(dataStore.truthPose(1:i,2),dataStore.truthPose(1:i,3),'b')
    plot(partTraj(:,1),partTraj(:,2),'r')
    
    drawparticlestar_red(xpartmean,ypartmean,tpartmean);
    drawparticlestar(dataStore.truthPose(i,2),dataStore.truthPose(i,3),dataStore.truthPose(i,4));
    
    firstnone = find(dataStore.timebeacon(i,:)==-1,1);
    beacnumb = (firstnone-1)/3;
    for j=1:beacnumb
        beacpoint = robot2global(dataStore.truthPose(i,2:4),dataStore.timebeacon(i,(j-1)*3+2:(j-1)*3+3));
        plot(beacpoint(1),beacpoint(2),'ro','MarkerFaceColor',[1 .6 .6]);
    end
    
    for j=1:beacnumb
        beacpoint = robot2global(partTraj(i,:),dataStore.timebeacon(i,(j-1)*3+2:(j-1)*3+3));
        plot(beacpoint(1),beacpoint(2),'ro','MarkerFaceColor',[1 0 .6]);
    end
    
    
    depthR = dataStore.rsdepth(i,3:end);
    depthXY = depth2xy(depthR, sensorOrigin);
    for j=1:9
        tempglob(j,:) = robot2global(dataStore.truthPose(i,2:4),[depthXY(1,j) depthXY(2,j)]);
    end
    plot(tempglob(:,1),tempglob(:,2),'*');
    
    for j=1:9
        tempglob(j,:) = robot2global(partTraj(i,:),[depthXY(1,j) depthXY(2,j)]);
    end
    plot(tempglob(:,1),tempglob(:,2),'*');
    
    x0=300;
    y0=300;
    width=720;
    height=600;
    set(gcf,'position',[x0,y0,width,height])
    xlim([-5, 5]);
    ylim([-4.5, 4.5]);
    
    
    title(['Time is ',num2str(i)])
    ax = gca;
    ax.Units = 'pixels';
    pos = ax.Position;
    ti = ax.TightInset;
    rect = [-ti(1), -ti(2), 1.05*(pos(3)+ti(1)+ti(3)), pos(4)+ti(2)+ti(4)];
    F = getframe(ax,rect);
    [A,map] = rgb2ind(frame2im(F),256);
    
    if i == 1
        imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',0.3);
    else
        imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',0.3);
    end
end