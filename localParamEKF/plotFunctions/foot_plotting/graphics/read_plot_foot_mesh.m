function [V8, F, FN8] = read_plot_foot_mesh(rightFoot, doPlot)
%PLOT_FOREARM_MESH Read the mesh of the forearm and project it to the
% reference frame 8, on the robot wrist, which the mesh is attached at.
% If required, plot the mesh.

if(rightFoot)
    forearm_file_name = './plotFunctions/foot_plotting/foot_right.obj';
else
    forearm_file_name = './plotFunctions/foot_plotting/foot_left.obj';
end
        
    [V,F,FN] = read_vertices_and_faces_from_obj_file(forearm_file_name);
    %V = V/10;   % convert to meters
    V = V/1000; % convert to meters

%     fprintf('Vertex: %d, Faces: %d\n', length(V), length(F));
    
    % define the transformation matrix from the reference frame 8 to the
    % cover reference frame C
    if(rightFoot)
        H_8C = eye(4);
        beta = 180*pi/180;
        % rotation around z
        Rx = [1 0 0 0
            0 cos(beta) -sin(beta) 0;
            0 sin(beta) cos(beta) 0;
            0 0 0 1];
        
        alpha = 90*pi/180;
        % rotation around z
        Rz = [cos(alpha) -sin(alpha) 0 0;
            sin(alpha) cos(alpha) 0 0;
            0 0 1 0; 0 0 0 1];
        R = Rz*Rx;

        
        H_8C = H_8C*R;
        
    else
        H_8C = eye(4);
        beta = 180*pi/180;
        % rotation around z
        Rx = [1 0 0 0
            0 cos(beta) -sin(beta) 0;
            0 sin(beta) cos(beta) 0;
            0 0 0 1];
        
        alpha = 90*pi/180;
        % rotation around z
        Rz = [cos(alpha) -sin(alpha) 0 0;
            sin(alpha) cos(alpha) 0 0;
            0 0 1 0; 0 0 0 1];
        R = Rz*Rx;

        
        H_8C = H_8C*R;
    end
    
    
    
    % now I can project the points of the cover to the frame 8 (wrist)
    V(:,4) = 1;
    FN(:,4) = 1;
    V8 = (H_8C * V')';
    FN8 = (H_8C * FN')';
    V8 = V8(:,1:3);
    FN8 = FN8(:,1:3);
    
    %remove duplicates
    [V8tmp, I, J] = unique(V8, 'rows');
    [m, n] = size(F);
    for i = 1 : m
        for j = 1 : n
            Ftmp(i,j) = J(F(i,j));
        end
    end
    F  = Ftmp;
    V8 = V8tmp;
    
    %take only the foot sole
    if rightFoot
        sole_heigth = 11.5e-3;
    else
        sole_heigth = 11.5e-3;
    end
    
    Fnew  = [];
    tol = 1e-3;
    for trg = 1: length(F)
        if (V8(F(trg,1),3) < sole_heigth+tol && V8(F(trg,2),3) < sole_heigth+tol && V8(F(trg,3),3) < sole_heigth+tol )
            if (V8(F(trg,1),3) > sole_heigth-tol && V8(F(trg,2),3) > sole_heigth-tol && V8(F(trg,3),3) > sole_heigth-tol )
                Fnew = [Fnew; F(trg,:)];
            end
        end
    end
    F = Fnew;
    
    %remove empty vertices
    Vnew = [];
    k = 1;
    for i = 1: size(V8,1)
        index = find(F(:)==i);
        if (~isempty(index))
            F(index)=k;
            Vnew(k,:) = V8(i,:);
            k = k+1;
        end
    end
    V8 = Vnew;        
    
%         
%     flat_index = find((V8(:,3) < V8(109,3) + 1e-3));
%     F = delaunay(V8(flat_index,1),V8(flat_index,2));
%     V8 = V8(flat_index,1:3);
    
    if(doPlot)
        trisurf(F,V8(:,1),V8(:,2),V8(:,3),'FaceColor',[0.26,0.33,1.0],'facealpha', 0.2);
%         trisurf(F,V(:,1),V(:,2),V(:,3),'FaceColor',[0.26,0.33,1.0 ],'facealpha', 0.5);
        % plot the face normals
%         hold on;
%         for i=1:length(FN)
%             vv = F(i,1);
%             quiver3(V8(vv,1), V8(vv,2), V8(vv,3), FN8(i,1), FN8(i,2), FN8(i,3));
%         end
        axis equal;
        xlabel('X');
        ylabel('Y');
        zlabel('Z');
    %     light('Position',[-1.0,-1.0,100.0],'Style','infinite');
    %     lighting phong;
    end
end


%%
function [V,F,FN] = read_vertices_and_faces_from_obj_file(filename)
  % Reads a .obj mesh file and outputs the vertex and face list
  % assumes a 3D triangle mesh and ignores everything but:
  % v x y z and f i j k lines
  % Input:
  %  filename  string of obj file's path
  %
  % Output:
  %  V  number of vertices x 3 array of vertex positions
  %  F  number of faces x 3 array of face indices
  %
  V = zeros(0,3);
  F = zeros(0,3);
  N = zeros(0,3);
  FN = zeros(0,3);
  vertex_index = 1;
  normal_index = 1;
  face_index = 1;
  fid = fopen(filename,'rt');
  line = fgets(fid);
  while ischar(line)
%       fprintf('line: %s\n', line);
    vertex = sscanf(line,'v %f %f %f');
    normal = sscanf(line,'vn %f %f %f');
%     face_long = sscanf(line,'f %d %d %d %d %d %d');
    face_long = sscanf(line,'f %d//%d %d//%d %d//%d');
%     face = sscanf(line,'f %d %d %d');

    % see if line is vertex command if so add to vertices
    if(size(vertex)>0)
      V(vertex_index,:) = vertex;
      vertex_index = vertex_index+1;
    % see if line is simple face command if so add to faces
%     elseif(length(face)>2)
%       if(face_index<10)
%         fprintf('f %d %d %d\n', face(1), face(2), face(3));
%       end
%       F(face_index,:) = face;
%       face_index = face_index+1;
    elseif(size(normal)>0)
        N(normal_index,:) = normal;
        normal_index = normal_index+1;
    % see if line is a long face command if so add to faces
    elseif(length(face_long)>5)      
      F(face_index,:) = face_long(1:2:end);
      FN(face_index,:) = N(face_long(2),:);
      face_index = face_index+1;
%     else
%       fprintf('Ignored: %s, Facelong: %d\n',line, length(face_long));
    end

    line = fgets(fid);
  end
  fclose(fid);
end