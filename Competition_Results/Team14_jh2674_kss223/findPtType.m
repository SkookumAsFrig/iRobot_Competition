function type = findPtType(point, pointSet)

x = point(1);
y = point(2);
type = Inf;
for i = 1:size(pointSet,1)
   if x == pointSet(i,1) && y == pointSet(i,2)
        type = pointSet(i,3);
   end
end
end