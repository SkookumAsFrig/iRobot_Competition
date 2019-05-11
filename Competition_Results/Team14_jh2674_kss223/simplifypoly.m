function polygons = simplifypoly(map, radius)

obstacles = wall2polygon(map,radius);
[m,~] = size(obstacles);

polygons = [];
checklist = zeros(m,1);

for i=1:m-1
    for j=i+1:m
        currpoly = obstacles(i,:);
        currx = currpoly(1:2:end);
        curry = currpoly(1:2:end);
        nextpoly = obstacles(j,:);
        nextx = currpoly(1:2:end);
        nexty = currpoly(1:2:end);
        
        [xa, ya] = polybool(currx, curry, nextx, nexty, 2);
        if isempty(xa)
            dd
            checklist(i)
        end
    end
end

end