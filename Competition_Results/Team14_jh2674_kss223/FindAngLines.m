function angle = FindAngLines(origin,onepoint,otherpoint)

a = onepoint-origin;
b = otherpoint-origin;

costheta = dot(a,b)/(norm(a)*norm(b));

angle = acosd(costheta);

end