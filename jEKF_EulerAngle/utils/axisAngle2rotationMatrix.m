function [ R ] = axisAngle2rotationMatrix( axisAngle )
%AXISANGLE2ROTATIONMATRIX(axisAngle) This function takes as input an axis/angle
%representation of an orientation in the form [n_x n_y n_z th] and returns
%the corresponding rotation matrix computed via the Rodriguez formula as
%described in http://it.mathworks.com/help/physmod/sm/mech/gs/representations-of-body-orientation.html#f13-7625

nx = axisAngle(1);
ny = axisAngle(2);
nz = axisAngle(3);
th = axisAngle(4);

J = [ 0  -nz  ny;
      nz  0  -nx;
     -ny  nx  0 ];
 
R = eye(3) + J*sin(th) + J^2*(1 - cos(th)) ;
display(R);

end

