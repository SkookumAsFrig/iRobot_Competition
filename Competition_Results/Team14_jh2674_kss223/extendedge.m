function newedge = extendedge(oldedge, amount)

point1 = oldedge(1:2);
point2 = oldedge(3:4);
midpoint = (point1+point2)./2;

topvec = point1-midpoint;
bottomvec = point2-midpoint;
halflength = norm(topvec);
newpoint1 = (1+amount/halflength)*topvec+midpoint;
newpoint2 = (1+amount/halflength)*bottomvec+midpoint;

newedge = [newpoint1 newpoint2];

end