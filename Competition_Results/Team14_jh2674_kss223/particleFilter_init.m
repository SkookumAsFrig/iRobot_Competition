function [M_final, M_debug, initw] = particleFilter_init(M_init,ubar,zt,ctrl_handle,w_handle,offmap_handle,reinit_handle,eachpartsize)
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
M_final = zeros(M,N);

M_resize = eachpartsize;
M_debug = [];

numbWaypoints = M/M_resize;
initw = zeros(numbWaypoints,1);

for z=1:numbWaypoints
    Mt = zeros(M_resize,N);
    wt = zeros(M_resize,1);
    
    for i=1:M_resize
        xbar = M_init(i+((z-1)*eachpartsize),1:N-1);
        xtpred = feval(ctrl_handle,xbar,ubar); %ctrl function should add noise
        wt(i) = feval(w_handle,xtpred,zt);
        Mt(i,1:N-1) = xtpred;
    end
    
    initw(z) = sum(wt);
    
    %normalize weights
    if sum(wt)==0
        wt = ones(M_resize,1)/M_resize;
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
    
    c = zeros(M_resize,1);
    c(1) = wt(1);
    for l=2:M_resize
        c(l) = c(l-1) + wt(l);
    end
    
    unif = unifrnd(0,1/M_resize);
    k = 1;
    
    for j=1:M_resize
        while unif>c(k)
            k = k+1;
        end
        %     M_final(j,:) = [Mt(k,1:N-1) 1/M];
        M_final(j+((z-1)*eachpartsize),:) = Mt(k,:);
        unif = unif+1/M_resize;
    end
    M_debug = [M_debug; Mt];
end

end
