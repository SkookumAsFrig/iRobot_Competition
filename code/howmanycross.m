function [crossnumb,blocknumb] = howmanycross(robotPose, sensorOrigin, depthinfo, wall)

crossnumb = 0;
blocknumb = 0;
depthXY = depth2xy(depthinfo, sensorOrigin);
for j=1:9
    tempglob = robot2global(robotPose,[depthXY(1,j) depthXY(2,j)]);
    [isect,~,~,ua]= intersectPoint(tempglob(1),tempglob(2),...
        robotPose(1),robotPose(2),wall(1),wall(2),wall(3),wall(4));
    otherend = ua*norm(tempglob-robotPose(1:2));
    if isect 
        if otherend>0.1
            crossnumb = crossnumb+1;
        else
            blocknumb = blocknumb + 1;
        end
    end
end

end