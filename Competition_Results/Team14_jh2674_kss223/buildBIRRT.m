function [newV,newconnect_mat,cost,path,pathpoints,expath,expoint,expath2,expoint2] = buildBIRRT(workspacetxt,limits,sampling_handle,startloc,goalloc,stepsize,radius)
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

timeplot=0;

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
V2 = goalloc;
q = 1;
q2 = 1;

obsedge = [];

connect_mat = [];
connect_mat2 = [];

for i=1:m
    %     minind = find(workspace(i,:)==0,1,'first')-1;
    %     if isempty(minind)
    %         minind = 16;
    %     end
    minind = 4;
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
% qnew = NaN;
% qnew2 = NaN;
endsw = 0;
see_end = 0;

h = circle([(limits(3)-limits(1))/2 (limits(4)-limits(2))/2],0,100,'r-');
h2 = circle([(limits(3)-limits(1))/2 (limits(4)-limits(2))/2],0,100,'r-');

while ~isequal(V(end,:),V2(end,:))
    if numinput==1
        newp = feval(sampling_handle,limits);
        newp2 = feval(sampling_handle,limits);
    else
        newp = feval(sampling_handle,limits,sampleind);
        sampleind = sampleind+1;
        newp2 = feval(sampling_handle,limits,sampleind);
        sampleind = sampleind+1;
    end
    %first tree
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if endsw == 1 || see_end ==1
        newp = V2(end,:);
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
        %         inside = inpoly2(newp,NODE,EDGE);
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
                %                 inside = inpoly2(qnew,NODE,EDGE);
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
            pause( timeplot );
            
            if isequal(V(end,:),V2(end,:))
                break
            end
            
            distrankstart_end = zeros(q2,1);
            for i=1:q2
                comppt = V2(i,:);
                distrankstart_end(i) = norm(qnew-comppt);
                
                csw2 = 0;
                for r=1:o
                    nowedge = obsedge(r,:);
                    [isect2,~,~,~]= intersectPoint(qnew(1),qnew(2),...
                        comppt(1),comppt(2),nowedge(1),nowedge(2),nowedge(3),nowedge(4));
                    if isect2
                        csw2 = 1;
                        break
                    end
                end
                if csw2==0
                    see_end = 1;
                end
            end
            
            [B3,I3] = sort(distrankstart_end);
            closestpt_se = V2(I3(1),:);
            closestdist_se = B3(1);
            
            if closestdist_se<=stepsize
                endsw = 1;
            end
        end
        
    end
    %second tree
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if endsw == 1 || see_end ==1
        newp2 = V(end,:);
        endsw = 0;
    end
    insw2 = 0;
    for i=1:m
        x = workspace(i,1:2:minind)';
        y = workspace(i,2:2:minind)';
        NODE = [x y];
        EDGE = [1:edgenum;[2:edgenum 1]]';
        %         inside2 = inpoly2(newp2,NODE,EDGE);
        inside2 = 0;
        csw5 = 0;
        for r=1:o
            nowedge = obsedge(r,:);
            isect5 = circle_lineseg(newp2(1),newp2(2),radius,...
                nowedge(1),nowedge(2),nowedge(3),nowedge(4));
            if isect5
                csw5 = 1;
                break
            end
        end
        if inside2 || csw5==1
            insw2 = 1;
            break
        end
    end
    if ~insw2
        [q2,~] = size(V2);
        distrank2 = zeros(q2,1);
        if see_end~=1
            for j=1:q2
                comppt2 = V2(j,:);
                distrank2(j) = norm(newp2-comppt2);
            end
            [B2,I2] = sort(distrank2);
            closestpt2 = V2(I2(1),:);
            closestdist2 = B2(1);
            plotindex2 = I2(1);
        else
            closestpt2 = V2(end,:);
            closestdist2 = norm(newp2-closestpt2);
            plotindex2 = q2;
            see_end = 0;
        end
        if closestdist2<=stepsize
            qnew2 = newp2;
        else
            qnew2 = (stepsize/closestdist2)*(newp2-closestpt2) + closestpt2;
        end
        
        csw6 = 0;
        for r=1:o
            nowedge = obsedge(r,:);
            [isect6,~,~,~]= intersectPoint(qnew2(1),qnew2(2),...
                closestpt2(1),closestpt2(2),nowedge(1),nowedge(2),nowedge(3),nowedge(4));
            if isect6
                csw6 = 1;
                break
            end
        end
        
        if csw6==0
            for i=1:m
                x = workspace(i,1:2:minind)';
                y = workspace(i,2:2:minind)';
                NODE = [x y];
                EDGE = [1:edgenum;[2:edgenum 1]]';
                %inside = inpoly2(qnew2,NODE,EDGE);
                inside = 0;
                if inside
                    csw6 = 1;
                    break
                end
            end
        end
        
        if csw6==0
            for r=1:o
                nowedge = obsedge(r,:);
                isect6 = circle_lineseg(qnew2(1),qnew2(2),radius,...
                    nowedge(1),nowedge(2),nowedge(3),nowedge(4));
                if isect6
                    csw6 = 1;
                    break
                end
            end
        end
        
        if csw6==0
            V2 = [V2;qnew2];
            realdist2 = norm(qnew2-closestpt2);
            connect_mat2(plotindex2,q2+1) = realdist2;
            connect_mat2(q2+1,plotindex2) = realdist2;
            expath2 = plot([V2(plotindex2,1) V2(end,1)],[V2(plotindex2,2) V2(end,2)],'b--','LineWidth',1.5,'HandleVisibility','off');
            expoint2 = plot(V2(end,1),V2(end,2),'g*','HandleVisibility','off');
            delete(h2)
            h2 = circle(V2(end,:),radius,100,'r-');
            axis equal
            pause( timeplot );
            
            distrankstart_end2 = zeros(q,1);
            for i=1:q
                comppt3 = V(i,:);
                distrankstart_end2(i) = norm(qnew2-comppt3);
                
                csw8 = 0;
                for r=1:o
                    nowedge = obsedge(r,:);
                    [isect8,~,~,~]= intersectPoint(qnew2(1),qnew2(2),...
                        comppt3(1),comppt3(2),nowedge(1),nowedge(2),nowedge(3),nowedge(4));
                    if isect8
                        csw8 = 1;
                        break
                    end
                end
                if csw8==0
                    see_end = 1;
                end
            end
            
            [B4,I4] = sort(distrankstart_end2);
            closestpt_se2 = V(I4(1),:);
            closestdist_se2 = B4(1);
            
            if closestdist_se2<=stepsize
                endsw = 1;
            end
        end
        
    end
end

[m2,~] = size(V);
[m3,~] = size(V2);
newV = [V;V2];
newconnect_mat = zeros(m2+m3,m2+m3);
newconnect_mat(1:m2,1:m2) = connect_mat;
newconnect_mat((m2+1):end,(m2+1):end) = connect_mat2;
placeholderval = 1e-10;
newconnect_mat(m2,end) = placeholderval;
newconnect_mat(end,m2) = placeholderval;
[cost,path] = dijkstra(newconnect_mat,newV,1,m2+1);
cost = cost - placeholderval;
pathpoints = newV(path,:);

end