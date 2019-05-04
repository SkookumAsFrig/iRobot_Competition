function [cost,path,vert] = findPath(workspacetxt,limits,initialp,goalp)
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
%       G               MATLAB object, weighted graph.
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 8
%   SHI, KOWIN

start_goal = [initialp;goalp];
[vert, connect_mat] = createRoadmap(workspacetxt,limits,start_goal);
[m,~] = size(vert);
[cost,path] = dijkstra(connect_mat,vert,m-1,m);

end