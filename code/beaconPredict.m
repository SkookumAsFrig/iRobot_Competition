function [canSee, coordinates] = beaconPredict(Pose,sensorOrigin,Walls,WallNum,beaconLoc,beaconNum)

xyG = robot2global([Pose(1) Pose(2) Pose(3)],sensorOrigin);

xt = Pose(1) + xyG(1);
yt = Pose(2) + xyG(2);
thetat = Pose(3);
anglimit = 25;

canSee = zeros(beaconNum,1);
coordinates = zeros(beaconNum,2);

for i=1:beaconNum
    nowbeacon = beaconLoc(i,:);
    origin = [xt yt];
    onepoint = [xt + xyG(1) yt + xyG(2)];
    otherpoint = [nowbeacon(2) nowbeacon(3)];
    angle = FindAngLines(origin,onepoint,otherpoint);
    
    if(abs(angle) < 25)
        isect = 0;
        for j=1:WallNum
            [isect,~,~,~]= intersectPoint(xt,yt,...
                nowbeacon(2),nowbeacon(3),Walls(j,1),Walls(j,2),Walls(j,3),Walls(j,4));
            if isect
                break
            end
        end
        
        if ~isect
            canSee(i) = 1;
            xyR = global2robot([Pose(1) Pose(2) Pose(3)],otherpoint);
            coordinates(i,:) = xyR;
        end
    end
    
end

end