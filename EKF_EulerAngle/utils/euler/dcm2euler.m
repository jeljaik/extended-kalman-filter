function [ phi ] = dcm2euler( R )
%DCM2EULER Converts a ZYZ Euler angle into a DCM matrix
%   The rotation described by ZYZ angles is obtained as composition of the
%   following elementary rotatoins: 
%   1. Rotate the reference frame by the angle alpha about axis z; this
%   rotation is described by the matrix Rz.
%   2. Rotate the current frame by the angle upsilon about axis y'; this
%   rotation is described by the matrix Ryy.
%   3. Rotate the current frame by the angle psi about axis zz; this
%   rotation is described by the matrix Rzz. 
%   The resulting frame orientation is obtained by composition of rotatoins
%   with respect to current frames, and then it can be computed via
%   postmultiplication of the matrices of elementary rotation.
%   R(phi) = Rz*Ryy*Rzz. Details on the structure of this matrix inside the
%   code.
%
%    alpha   = atan2(r23,r13);
%    upsilon = atan2(sqrt(r13^2 + r23^2),r33);
%    psi     = atan2(r32,-r31);


r21 = R(2,1);
r23 = R(2,3);
r11 = R(1,1);
r13 = R(1,3);
r33 = R(3,3);
r32 = R(3,2);
r31 = R(3,1);

 alpha   = atan2(r23,r13);
 upsilon = atan2(sqrt(r13^2 + r23^2),r33);
 psi     = atan2(r32,-r31);

%alpha   = atan2(r21,r11);
%upsilon = atan2(-r31, sqrt(r32^2 + r33^2));
%psi     = atan2(r32, r33);

phi = [alpha; upsilon; psi];
end

