function [M_final, Mt] = particleFilter(M_init,ubar,zt,ctrl_handle,w_handle,offmap_handle,reinit_handle)
% hGPS: predict the depth measurements for a robot operating
% in a known map, given the expected range measurements.
%
%   depth = depthPredict(angles, range) returns
%   the expected depth measurements for a robot given range measurements.
%
%   INPUTS
%       angles      	K-by-1 vector of the angular positions of the range
%                   	sensor(s) in robot coordinates, where 0 points forward
%       range           K-by-1 vector of ranges (meters)
%
%   OUTPUTS
%       depth       	K-by-1 vector of depths (meters)
%
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 4
%   SHI, KOWIN

[M,N] = size(M_init);

Mt = zeros(M,N);
wt = zeros(M,1);
for i=1:M
    xbar = M_init(i,1:N-1);
    xtpred = feval(ctrl_handle,xbar,ubar); %ctrl function should add noise
    wt(i) = feval(w_handle,xtpred,zt);
    Mt(i,1:N-1) = xtpred;
end

%normalize weights
if sum(wt)==0
    wt = ones(M,1)/M;
end

Mt(:,end) = wt;
wt = feval(offmap_handle,Mt);

if wt(1)==-1 && wt(2)==-1
    Mt = feval(reinit_handle);
    wt = Mt(:,end);
end

wt = wt./sum(wt);
Mt(:,end) = wt;

%prediction step done
%update step:
M_final = zeros(M,N);
c = zeros(M,1);
c(1) = wt(1);
for l=2:M
    c(l) = c(l-1) + wt(l);
end

unif = unifrnd(0,1/M);
k = 1;

for j=1:M
    while unif>c(k)
        k = k+1;
    end
    %     M_final(j,:) = [Mt(k,1:N-1) 1/M];
    M_final(j,:) = Mt(k,:);
    unif = unif+1/M;
end

end
