clc
clear all
close all

% load('biRRT_bigtestrun3.mat')
% map = 'compMap_big.mat';

load('RRT_testrun3.mat')
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
numpart = 250;
sensorOrigin = [0.13 0];
angles = linspace(27*pi/180,-27*pi/180,9);

partTraj = [];

deadreck = dataStore.truthPose(1,2:4);

filename = 'testAnimated_fixed.gif';

seeBeacInd = 1;

for i=1:length(dataStore.truthPose(:,1))
    clf
    disp(i)
    hold on
    axis equal
    for j=1:l
        plot([mapdata(j,1) mapdata(j,3)],[mapdata(j,2) mapdata(j,4)],'LineWidth',2,'Color','k','HandleVisibility','off')
    end
    for e=1:o
        plot(beaconmat(e,2),beaconmat(e,3),'rp','MarkerFaceColor','r')
        text(beaconmat(e,2),beaconmat(e,3),num2str(beaconmat(e,1)))
    end
    
    dvec = dataStore.odometry(i,2);
    phivec = 1.1*dataStore.odometry(i,3);
    
    if dataStore.timebeacon(i,1)~=-1
        seeBeacInd = seeBeacInd+1;
    else
        seeBeacInd = 1;
    end
    
    if seeBeacInd>10
        xi = mean(dataStore.particles(:,1,i-1));
        yi = mean(dataStore.particles(:,2,i-1));
        thetai = mean(dataStore.particles(:,3,i-1));
        seeBeacInd = 1;
    else
        xi = deadreck(i,1);
        yi = deadreck(i,2);
        thetai = deadreck(i,3);
    end
    
    newstate = integrateOdom_onestep(xi, yi, thetai, dvec, phivec);
    deadreck = [deadreck; newstate'];
    
    if initsw == 0
        oriPose = dataStore.truthPose(1,2:4);
        
        initsw = 1;
        %% PF
        
        xpart = xrange*rand(numpart,1)+minX;
        ypart = yrange*rand(numpart,1)+minY;
        thepart = 2*pi*rand(numpart,1);
        
        wi = ones(numpart,1)/numpart;
        dataStore.particles = [xpart ypart thepart wi];
    else
        
        ctrl = [dvec phivec];
        zt = [dataStore.rsdepth(i,3:end) dataStore.timebeacon(i,:) deadreck(i,:)]';
        
        %% PF
        M_init = dataStore.particles(:,:,end);
        ctrl_handle = @(mubar,ubar) integrateOdom_onestep_wnoise(mubar(1),...
            mubar(2),mubar(3),ubar(1),ubar(2));
        w_handle = @(xtpred,meas) findWeight(xtpred,mapdata,sensorOrigin,...
            angles,meas,beaconmat);
        o_handle = @(PSet) offmap_detect(PSet,mapdata);
        reinit_handle = @() resample(mapdata,numpart);
        [M_final,Mt]= particleFilter(M_init,ctrl,zt,ctrl_handle,w_handle,o_handle,reinit_handle);
        dataStore.particles = cat(3,dataStore.particles,M_final);
        
    end
    
    %     for k=1:3:numpart
    %         drawparticle(dataStore.particles(k,1,i),dataStore.particles(k,2,i),dataStore.particles(k,3,i));
    %     end
    
    xpartmean = mean(dataStore.particles(:,1,i));
    ypartmean = mean(dataStore.particles(:,2,i));
    tpartmean = mean(dataStore.particles(:,3,i));
    partTraj = [partTraj; [xpartmean ypartmean tpartmean]];
    
    plot(dataStore.truthPose(1:i,2),dataStore.truthPose(1:i,3),'b')
    plot(deadreck(1:i,1),deadreck(1:i,2),'g')
    %plot(partTraj(:,1),partTraj(:,2),'r')
    
    drawparticlestar_red(xpartmean,ypartmean,tpartmean);
    
    %plot(dataStore.truthPose(i,2),dataStore.truthPose(i,3),'rp','MarkerSize',20)
    drawparticlestar(dataStore.truthPose(i,2),dataStore.truthPose(i,3),dataStore.truthPose(i,4));
    drawparticlestar_green(deadreck(i,1),deadreck(i,2),deadreck(i,3));
    title(['Time is ',num2str(i)])
    ax = gca;
    ax.Units = 'pixels';
    pos = ax.Position;
    ti = ax.TightInset;
    rect = [-ti(1), -ti(2), 1.05*(pos(3)+ti(1)+ti(3)), pos(4)+ti(2)+ti(4)];
    F = getframe(ax,rect);
    [A,map] = rgb2ind(frame2im(F),256);
    
    if i == 1
        imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',0.1);
    else
        imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',0.1);
    end
end