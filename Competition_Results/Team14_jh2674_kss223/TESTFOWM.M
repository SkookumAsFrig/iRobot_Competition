map = 'compMap_mod.mat';
mapstruct = importdata(map);
optWalls = mapstruct.optWalls;
for i = 1:size(optWalls,1)
    line([optWalls(i,1),optWalls(i,3)],[optWalls(i,2),optWalls(i,4)],'color','b');
    hold on;
end
midpoints = findOptWallMidpoint(optWalls);