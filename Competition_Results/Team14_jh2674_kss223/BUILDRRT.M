function [V,connect_mat,cost,path,pathpoints,expath,expoint] = buildRRT(workspacetxt,limits,sampling_handle,startloc,goalloc,stepsize,radius)
% findPath: given a polygonal environment, a roadmap, and initial and goal
% points, returns the shortest path connecting the initial and goal points.
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



if ~contains(workspacetxt,'txt')
    workspace = importdata(workspacetxt);
    workspace = workspace.map;
else
    fileID = fopen(workspacetxt,'r');
    formatSpec = '%f';
    workspace = fscanf(fileID,formatSpec,[16 Inf])';
end

[m,n] = size(workspace);
if n==4
    workspace = [workspace zeros(m,1)];
end


V = startloc;

obsedge = [];
connect_mat = [];

for i=1:m
    %minind = find(workspace(i,:)==0,1,'first')-1;
    minind = 4;
%     if isempty(minind)
%         minind = 16;
%     end
    for j=1:minind/2-1
        obsedge = [obsedge;workspace(i,2*j-1:2*j+2)];
    end
    obsedge = [obsedge;[workspace(i,minind-1:minind)...
        workspace(i,1:2)]];
end

if n~=4
    boundaryvec = [limits(1) limits(2) limits(1) limits(4);...
        limits(1) limits(4) limits(3) limits(4);...
        limits(3) limits(4) limits(3) limits(2);...
        limits(3) limits(2) limits(1) limits(2)];
    obsedge = [obsedge;boundaryvec];
end

[o,~] = size(obsedge);

numinput = nargin(sampling_handle);
sampleind = 1;
qnew = NaN;
endsw = 0;
see_end = 0;

h = circle([(limits(3)-limits(1))/2 (limits(4)-limits(2))/2],0,100,'r-');

while ~isequal(qnew,goalloc)
    if numinput==1
        newp = feval(sampling_handle,limits);
    else
        newp = feval(sampling_handle,limits,sampleind);
        sampleind = sampleind+1;
    end
    if endsw == 1 || see_end ==1
        newp = goalloc;
        endsw = 0;
    end
    insw = 0;
    for i=1:m
%         minind = find(workspace(i,:)==0,1,'first')-1;
%         if isempty(minind)
%             minind = 16;
%         end
        edgenum = minind/2;
        x = workspace(i,1:2:minind)';
        y = workspace(i,2:2:minind)';
        NODE = [x y];
        EDGE = [1:edgenum;[2:edgenum 1]]';
        %inside = inpoly2(newp,NODE,EDGE);
        inside = 0;
        csw1 = 0;
        for r=1:o
            nowedge = obsedge(r,:);
            isect1 = circle_lineseg(newp(1),newp(2),radius,...
                nowedge(1),nowedge(2),nowedge(3),nowedge(4));
            if isect1
                csw1 = 1;
                break
            end
        end
        if inside || csw1==1
            insw = 1;
            break
        end
    end
    if ~insw
        [q,~] = size(V);
        distrank = zeros(q,1);
        if see_end~=1
            for j=1:q
                comppt = V(j,:);
                distrank(j) = norm(newp-comppt);
            end
            [B,I] = sort(distrank);
            closestpt = V(I(1),:);
            closestdist = B(1);
            plotindex = I(1);
        else
            closestpt = V(end,:);
            closestdist = norm(newp-closestpt);
            plotindex = q;
            see_end = 0;
        end
        if closestdist<=stepsize
            qnew = newp;
        else
            qnew = (stepsize/closestdist)*(newp-closestpt) + closestpt;
        end
        
        csw = 0;
        for r=1:o
            nowedge = obsedge(r,:);
            [isect,~,~,~]= intersectPoint(qnew(1),qnew(2),...
                closestpt(1),closestpt(2),nowedge(1),nowedge(2),nowedge(3),nowedge(4));
            if isect
                csw = 1;
                break
            end
        end
        
        if csw==0
            for i=1:m
                x = workspace(i,1:2:minind)';
                y = workspace(i,2:2:minind)';
                NODE = [x y];
                EDGE = [1:edgenum;[2:edgenum 1]]';
                %inside = inpoly2(qnew,NODE,EDGE);
                inside = 0;
                if inside
                    csw = 1;
                    break
                end
            end
        end
        
        if csw==0
            for r=1:o
                nowedge = obsedge(r,:);
                isect3 = circle_lineseg(qnew(1),qnew(2),radius,...
                    nowedge(1),nowedge(2),nowedge(3),nowedge(4));
                if isect3
                    csw = 1;
                    break
                end
            end
        end
        
        if csw==0
            V = [V;qnew];
            realdist = norm(qnew-closestpt);
            connect_mat(plotindex,q+1) = realdist;
            connect_mat(q+1,plotindex) = realdist;
            expath = plot([V(plotindex,1) V(end,1)],[V(plotindex,2) V(end,2)],'r--','LineWidth',1.5,'HandleVisibility','off');
            expoint = plot(V(end,1),V(end,2),'k*','HandleVisibility','off');
            delete(h)
            h = circle(V(end,:),radius,100,'r-');
            axis equal
            pause( 0.01 );
            
            if norm(qnew-goalloc)<=stepsize
                endsw = 1;
            end
            csw2 = 0;
            for r=1:o
                nowedge = obsedge(r,:);
                [isect2,~,~,~]= intersectPoint(qnew(1),qnew(2),...
                    goalloc(1),goalloc(2),nowedge(1),nowedge(2),nowedge(3),nowedge(4));
                if isect2
                    csw2 = 1;
                    break
                end
            end
            if csw2==0
                see_end = 1;
            end
        end
        
    end
end

[m2,~] = size(V);
[cost,path] = dijkstra(connect_mat,V,1,m2);
pathpoints = V(path,:);

end
