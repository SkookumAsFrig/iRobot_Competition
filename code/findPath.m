function [cost,path,vert] = findPath(polygons,limits,waypoints,start,mapdata,extendamount)
% findPath: given a polygonal environment, a roadmap, and initial and goal
% points, returns the shortest path connecting the initial and goal points.

% Uses: Dijkstra's Minimum Cost Path Algorithm
% By: Joseph Kirk
% jdkirk630@gmail.com
% link : https://www.mathworks.com/matlabcentral/fileexchange/20025-dijkstra-s-minimum-cost-path-algorithm?s_tid=FX_rc1_behav
%
%   depth = depthPredict(angles, range) returns
%   the expected depth measurements for a robot given range measurements.
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
%      limits           Boundary of the workspace, given in [xmin ymin xmax ymax]
%
%   OUTPUTS
%       vert            N-by-2 vector of nodes, with each row being [x y]
%       connect_mat     N-by-N connectivity matrix, diagonal is
%                       ignored (all 0) because self loops are meaningless
%                       in this context. The values (if not 0) are the edge
%                       weights based on distance.
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 8
%   SHI, KOWIN

[m,n] = size(mapdata);
mapext = zeros(m-4,n);
for i=1:m-4
    mapext(i,:) = extendedge(mapdata(i+4,:), extendamount);
end

g = size(polygons,1);
edges = zeros(g*4,4);
for i=1:g
    for j=1:4
        edges(4*(i-1)+j,:) = polygons(i,2*(j-1)+1:2*(j-1)+4);
    end
end

[vert, connect_mat] = createRoadmap(polygons,limits,waypoints,start,mapext,edges);
[m,~] = size(vert);
[n,~] = size(waypoints);
cost = inf;
for i=1:n
    [realcost,realpath] = dijkstra(connect_mat,vert,m,m-n-1+i);
    if realcost<cost
        path = realpath;
        cost = realcost;
    end
end

end