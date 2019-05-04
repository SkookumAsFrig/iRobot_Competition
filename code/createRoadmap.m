function [vert, connect_mat] = createRoadmap(polygons,limits,waypoints,start, mapextend, edges)
% createRoadmap: Given a polygonal environment returns a roadmap covering
% Q_free. Can also input initial and goal points to include in roadmap.
%
%   [vert, connect_mat, G] = createRoadmap(polygonstxt,limits,start_goal)
%   returns the Nodes, Connectivity Matrix and MATLAB Graph Object
%   representing the roadmap.
%
%   INPUTS
%      polygonstxt     Each line in the file contains the vertices of
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

for i=1:m
    x = polygons(i,1:2:end)';
    y = polygons(i,2:2:end)';
    for j=1:4
        currx = x(j);
        curry = y(j);
        if currx>limits(1) && curry>limits(2) && currx<limits(3) && curry<limits(4)
            vert = [vert;[currx curry]];
        end
    end
end

vert = [vert; waypoints; start];

[g,~] = size(vert);
[n,~] = size(mapextend);
[p,~] = size(edges);
connect_mat = zeros(g,g);

for j=1:g-1
    currvert = vert(j,1:2);
    for z=j+1:g
        endvert = vert(z,1:2);
        desw = 0;
        for l=1:n
            [isect,~,~,~]= intersectPoint(currvert(1),currvert(2),...
                endvert(1),endvert(2),mapextend(l,1),mapextend(l,2),...
                mapextend(l,3),mapextend(l,4));
            if isect
                desw = 1;
                break
            end
        end
        if desw==0
            for l=1:p
                doesntcount = 0;
                [isect,~,~,ua]= intersectPoint(currvert(1),currvert(2),...
                    endvert(1),endvert(2),edges(l,1),edges(l,2),...
                    edges(l,3),edges(l,4));
                if isect && ua>1e-7 && abs(ua-1)>1e-7
                    if z==g
                        pind = ceil(l/4);
                        x = polygons(pind,1:2:8)';
                        y = polygons(pind,2:2:8)';
                        NODE = [x y];
                        EDGE = [1:4;[2:4 1]]';
                        inside = inpoly2(endvert,NODE,EDGE);
                        if inside
                            doesntcount = 1;
                        end
                    end
                    if doesntcount == 0
                        desw = 1;
                        break
                    end
                end
            end
        end
        if desw==0
            dist = norm(currvert-endvert);
            connect_mat(j,z)=dist;
            connect_mat(z,j)=dist;
        end
    end
    
end

end