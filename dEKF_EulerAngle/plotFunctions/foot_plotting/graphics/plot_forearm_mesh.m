function [V8, F] = plot_forearm_mesh(forearm_file_name)
%PLOT_FOREARM_MESH Plot the mesh of the forearm
    if( isempty(forearm_file_name))
        forearm_file_name = '../data/coverMesh/forearm_left.obj';
    end
    

    [V,F] = read_vertices_and_faces_from_obj_file(forearm_file_name);
    V = V/1000;   % convert to meters
    fprintf('Vertex: %d, Faces: %d\n', length(V), length(F));
    
    % define the transformation matrix from the reference frame 8 to the
    % cover reference frame C
    H_7C = eye(4); 
    temp = H_7C(:,1); H_7C(:,1)=H_7C(:,2); H_7C(:,2)=temp;
    H_7C(1,4)=-0.005; H_7C(3,4)=0.08;
    H_78 = evalDHMatrix(   0,   0.1373,     pi/2, -pi/2);
    H_8C = Hinv(H_78)*H_7C;

    % now I can project the points of the cover to the frame 8 (wrist)
    V(:,4) = 1;
    V8 = zeros(size(V));
    for i=1:size(V,1)
       V8(i,:) = H_8C * V(i,:)'; 
    end
    
    trisurf(F,V8(:,1),V8(:,2),V8(:,3),'FaceColor',[0.26,0.33,1.0 ]);
    axis equal;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
%     light('Position',[-1.0,-1.0,100.0],'Style','infinite');
%     lighting phong;
end


%%
function [V,F] = read_vertices_and_faces_from_obj_file(filename)
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
  vertex_index = 1;
  face_index = 1;
  fid = fopen(filename,'rt');
  line = fgets(fid);
  while ischar(line)
%       fprintf('line: %s\n', line);
    vertex = sscanf(line,'v %f %f %f');    
    face_long = sscanf(line,'f %d %d %d %d %d %d');
    %face_long = sscanf(line,'f %d//%d %d//%d %d//%d');
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
    % see if line is a long face command if so add to faces
    elseif(length(face_long)>5)
      % remove normal indices
      face_long = face_long(1:2:end);
      F(face_index,:) = face_long;
      face_index = face_index+1;
%     else
%       fprintf('Ignored: %s, Facelong: %d\n',line, length(face_long));
    end

    line = fgets(fid);
  end
  fclose(fid);
end