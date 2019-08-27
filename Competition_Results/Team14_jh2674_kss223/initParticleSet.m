function [xpart,ypart,thepart,wi] = initParticleSet(waypoints,eachnumpart,wptsNum,numpart)

xprep = repmat(waypoints(:,1),1,eachnumpart);
xpart = reshape(xprep',[],1);
yprep = repmat(waypoints(:,2),1,eachnumpart);
ypart = reshape(yprep',[],1);
tprep = linspace(2*pi/eachnumpart,2*pi,eachnumpart);
thepart = repmat(tprep,1,wptsNum)';
wi = ones(numpart,1)/numpart;

end