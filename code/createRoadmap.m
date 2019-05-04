function [vert, connect_mat] = createRoadmap(polygons,limits,start_goal)
% createRoadmap: Given a polygonal environment returns a roadmap covering
% Q_free. Can also input initial and goal points to include in roadmap.
%
%   [vert, connect_mat, G] = createRoadmap(workspacetxt,limits,start_goal) 
%   returns the Nodes, Connectivity Matrix and MATLAB Graph Object
%   representing the roadmap.
%
%   INPUTS
%      workspacetxt     Each line in the file contains the vertices of
%                       one polygonal obstacle: v1x, v1y, v2x, v2y,
%                       etc. The vertices are given in counterclockwise
%                       order. To make it easy to import in MATLAB,
%                       each line contains 16 entries corresponding to
%                       (up to) 8 vertices. If an obstacle has fewer
%                       vertices, unused entries in the line will
%                       contain the value zero.
%      limits           Boundary of the polygons, given in [xmin ymin xmax ymax]
%      start_goal       Optional input, for [start point; goal point] (2x2)
%
%   OUTPUTS
%       vert            N-by-2 vector of nodes, with each row being [x y]
%       connect_mat     N-by-N connectivity matrix, diagonal is
%                       ignored (all 0) because self loops are meaningless
%                       in this context. The values (if not 0) are the edge
%                       weights based on distance.
%       G               MATLAB object, weighted graph.
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 8
%   SHI, KOWIN

[m,~] = size(polygons);

vert = [];
edges = cell(m,1);
convhullres = cell(m,1);

for i=1:m
    minind = 10;
    x = polygons(i,1:2:minind)';
    y = polygons(i,2:2:minind)';
    [lv,~] = size(vert);
    convind = [1:length(x)-1 1];
    convind = convind + lv;
    convhullres{i} = [x y convind'];
    vert = [vert;[x(1:end-1) y(1:end-1) i*ones(length(x)-1,1)]];
    tempedge = [];
    for j=1:4
        tempedge = [tempedge;polygons(i,2*j-1:2*j+2)];
    end
    edges{i}=tempedge;
end

% bounds = [limits(1) limits(2);limits(1) limits(4);limits(3) limits(2);limits(3) limits(4)];
% 
% vert = [vert;[bounds zeros(4,1)]];
if nargin==3
    vert = [vert;[start_goal -ones(2,1)]];
end

[g,~] = size(vert);
connect_mat = zeros(g,g);

for j=1:g-1
    currvert = vert(j,1:2);
    for z=j+1:g
        endvert = vert(z,1:2);
        desw = 0;
        
        for k=1:m
            edgevec = edges{k};
            [n,~]=size(edgevec);
            if ismember([currvert endvert],edgevec)
                desw = 1;
                break
            end
            for l=1:n
                [isect,~,~,ua]= intersectPoint(currvert(1),currvert(2),...
                    endvert(1),endvert(2),edgevec(l,1),edgevec(l,2),...
                    edgevec(l,3),edgevec(l,4));
                if isect && ua>1e-7 && abs(ua-1)>1e-7
                    desw = 1;
                    break
                end
            end
            if desw==1
                break
            end
        end
        if desw==0
            dist = norm(currvert-endvert);
            connect_mat(j,z)=dist;
            connect_mat(z,j)=dist;
        end
    end
end

for i=1:m
    nowconv = convhullres{i};
    [h,~] = size(nowconv);
    for j=1:h-1
        thisind = nowconv(j,3);
        nextind = nowconv(j+1,3);
        dist = norm(nowconv(j,1:2)-nowconv(j+1,1:2));
        connect_mat(thisind,nextind)=dist;
        connect_mat(nextind,thisind)=dist;
    end
end

for i=1:g-3
    %within same polygon
    thisobs = vert(i,3);
    nextobs = vert(i+1,3);
    if thisobs == nextobs
        dist = norm(vert(i,1:2)-vert(i+1,1:2));
        connect_mat(i,i+1)=dist;
        connect_mat(i+1,i)=dist;
    end
end

end
