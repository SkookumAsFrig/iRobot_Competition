function crossnumb = howmanycross(robotPose, sensorOrigin, depthinfo, wall)

crossnumb = 0;

depthXY = depth2xy(depthinfo, sensorOrigin);
for j=1:9
    tempglob = robot2global(robotPose,[depthXY(1,j) depthXY(2,j)]);
    
end



end