function [ R ] = euler2dcm( phi )
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
%    R    =  [cos(alpha)*cos(upsilon)*cos(psi)-sin(alpha)*sin(psi)   -cos(alpha)*cos(upsilon)*sin(psi)-sin(alpha)*cos(psi)   cos(psi)*sin(upsilon);...
%              sin(alpha)*cos(upsilon)*cos(psi)+cos(alpha)*sin(psi)   -sin(alpha)*cos(upsilon)*sin(psi)+cos(alpha)*cos(psi)   sin(alpha)*sin(upsilon);...
%                        -sin(upsilon)*cos(psi)                                   sin(upsilon)*sin(psi)                       cos(upsilon)]

alpha=phi(1);
upsilon=phi(2);
psi=phi(3);

R    =  [cos(alpha)*cos(upsilon)*cos(psi)-sin(alpha)*sin(psi)   -cos(alpha)*cos(upsilon)*sin(psi)-sin(alpha)*cos(psi)   cos(psi)*sin(upsilon);...
         sin(alpha)*cos(upsilon)*cos(psi)+cos(alpha)*sin(psi)   -sin(alpha)*cos(upsilon)*sin(psi)+cos(alpha)*cos(psi)   sin(alpha)*sin(upsilon);...
                   -sin(upsilon)*cos(psi)                                   sin(upsilon)*sin(psi)                       cos(upsilon)];


end

