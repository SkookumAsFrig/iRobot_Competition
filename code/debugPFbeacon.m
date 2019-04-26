clc
clear all
close all

load('RRT_testrun.mat')
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
numpart = 200;
sensorOrigin = [0.13 0];
angles = linspace(27*pi/180,-27*pi/180,9);

for i=1:length(dataStore.truthPose(:,1))
    clf
    disp(i)
    hold on
    axis equal
    %plot(dataStore.truthPose(i,2),dataStore.truthPose(i,3),'rp','MarkerSize',20)
    drawparticlestar(dataStore.truthPose(i,2),dataStore.truthPose(i,3),dataStore.truthPose(i,4));
    for j=1:l
        plot([mapdata(j,1) mapdata(j,3)],[mapdata(j,2) mapdata(j,4)],'LineWidth',2,'Color','k','HandleVisibility','off')
    end
    for e=1:o
        plot(beaconmat(e,2),beaconmat(e,3),'rp','MarkerFaceColor','r')
    end
    
    dvec = dataStore.odometry(i,2);
    phivec = dataStore.odometry(i,3);
    
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
        zt = [dataStore.rsdepth(i,3:end) dataStore.beacon(i,3:5)]';
        
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
    for k=1:numpart
        drawparticle(dataStore.particles(k,1,i),mean(dataStore.particles(k,2,i)),mean(dataStore.particles(k,3,i)));
    end
end