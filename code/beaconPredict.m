function [canSee, coordinates] = beaconPredict(Pose,sensorOrigin,Walls,WallNum,beaconLoc,beaconNum)

xyG = robot2global([Pose(1) Pose(2) Pose(3)],sensorOrigin);

xt = xyG(1);
yt = xyG(2);
thetat = Pose(3);
anglimit = 25;

canSee = zeros(beaconNum,1);
coordinates = zeros(beaconNum,2);

for i=1:beaconNum
    xyG2 = robot2global([xyG Pose(3)],[1 0]);
    nowbeacon = beaconLoc(i,:);
    origin = [xt yt];
    onepoint = xyG2;
    otherpoint = [nowbeacon(2) nowbeacon(3)];
    angleval = FindAngLines(origin,onepoint,otherpoint);
    
    if(abs(angleval) < anglimit)
        isect = 0;
        realcross = 0;
        for j=1:WallNum
            [isect,~,~,ua]= intersectPoint(xt,yt,...
                nowbeacon(2),nowbeacon(3),Walls(j,1),Walls(j,2),Walls(j,3),Walls(j,4));
            if isect && (ua<0.95)
                realcross = 1;
                break
            end
        end
        
        if ~realcross
            canSee(i) = 1;
            xyR = global2robot([Pose(1) Pose(2) Pose(3)],otherpoint);
            coordinates(i,:) = xyR;
        end
    end
    
end

end