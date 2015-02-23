function [ pointsProj ] = projectPointOnMesh( points, V, F )
%PROJECTPOINTONMESH Project the specified 3D points on a mesh.
% The mesh is represented by:
% - V: a list of vertices
% - F: a list of triangular faces (3 indexes referring to V)
    fprintf('\n\nPROJECT POINT ON MESH\n');
%     points
%     V(1:5,:)
%     F(1:5,:)

    pointsProj = zeros(size(points));

    %% for every input point, find the closest vertex
    closestV = ones(size(points,1),1);
    for i=1:size(points,1)  % for every input point
        minDist = norm(points(i,:)-V(1,:));
        for j=2:size(V,1)   % for every mesh vertex
            dist = norm(points(i,:)-V(j,:));
            if(dist<minDist)
                closestV(i) = j;
                minDist = dist;
            end
        end
    end

    %% for every vertex, find all the faces containing it
    vertexToFaces = cell(size(V,1),1);
    for i=1:size(F,1)       % for every face
        for j=1:3   % for every face vertex
            vertexToFaces{F(i,j)}(end+1) = i;
        end
        vertexToFaces{F(i,j)} = 1:size(V,1);
    end
        
    %% for every input point, find the closest point on the mesh surface
    for i=1:size(points,1)          % for every input point
        
        %consider only the triangles with the closes vertex
        %faceList = vertexToFaces{closestV(i)};
        
        %do not consider only the triangles with the closes vertex
        faceList = 1:size(F,1);
        
        if (~isempty(faceList))
            %         fprintf('Point %d is close to vertex %d which belongs to %d faces\n',i,closestV(i),length(faceList));
            dist = zeros(size(faceList));
            P = zeros(length(faceList),3);
            for j=1:length(faceList)    % for every mesh face close to the point
                v0 = V(F(faceList(j),1),:);
                v1 = V(F(faceList(j),2),:);
                v2 = V(F(faceList(j),3),:);
                %             fprintf('Projecting point %d on triangle %d\n', i, faceList(j));
                [P(j,:), dist(j)] = projectPointOnTriangle(points(i,:), v0, v1, v2);
            end
            [~, minInd] = min(dist);    % find the index of the closest triangle
            pointsProj(i,:) = P(minInd,:);
        end
    end
end


%%
function [projectedP, distance] = projectPointOnTriangle(P, v0, v1, v2)
% Project a 3D point on a triangle.
% Find the closest point to P that lies on the triangle defined by v0, v1,
% v2.

    %% if the line that passes through P and is normal to the triangle
    %% intersects the triangle, then the intersection point is the closest    
    normal = cross(v0-v1, v1-v2);
    normal = normal / norm(normal); % unit vector    
    [flag, ~, ~, t] = rayTriangleIntersection(P, normal, v0, v1, v2);
    if(flag==1)
        projectedP = P+(t*normal);
        distance = norm(projectedP-P);
%         fprintf('Point projection is inside the triangle\n');
        return;
    end
%     fprintf('Point projection is on one of the triangle edge\n');
    
    %% the closest point lies on one of the triangle's edges
    % find the closest edge
    dist = zeros(3,1);
    edges = zeros(3,3);
    edgesMod = zeros(3,1);
    proj = zeros(3,1);
    V = [v0;v1;v2];
    edges(1,:)=v1-v0; edges(2,:)=v2-v1; edges(3,:)=v0-v2;
    edgesMod(1)=norm(edges(1,:)); 
    edgesMod(2)=norm(edges(2,:)); 
    edgesMod(3)=norm(edges(3,:));
    for i=1:3
        % projection of P on edge i
        proj(i) = (P-V(i,:))*edges(i,:)' / edgesMod(i);
        if(proj(i)<=0) % the closest point is the 1st vertex
            dist(i) = norm(P-V(i,:));
        elseif(proj(i)>=edgesMod(i)) % the closest point is the 2nd vertex
            dist(i) = norm(P-V(mod(i,3)+1,:));
        else    % the closest point is inside the segment
            dist(i) = norm(cross(edges(i,:), P-V(i,:))/edgesMod(i));
        end
    end
    [distance, minI] = min(dist);
    
    % compute the closest point to P, on the closest edge
    if(proj(minI)<=0)   %first vertex
        projectedP = V(minI,:);
    elseif(proj(minI)>=edgesMod(minI))  %second vertex
        projectedP = V(mod(minI,3)+1,:);
    else                % in between the 2 vertices
        projectedP = V(minI,:) + proj(minI)*(edges(minI,:)/edgesMod(minI));
    end
%     fprintf('closestV=[%f %f %f]\t', V(minI,1),V(minI,2),V(minI,3));
%     fprintf('proj=%f\tres=[%f %f %f]\n', proj(minI), projectedP(1),projectedP(2),projectedP(3));
end
