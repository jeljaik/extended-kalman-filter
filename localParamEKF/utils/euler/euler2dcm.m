function [ R ] = euler2dcm( phi, t )
%DCM2EULER Converts a ZYZ Euler angle into a DCM matrix
% EULER2DCM(PHI)      Returns the aforementioned DC rotation matrix.
% EULER2DCM(PHI, 'T') Time-series version of euler2dcm(Phi), i.e. Phi is a
% matrix of Nx3 elements where each row corresponds to a value of Phi at
% time k.
%
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
% Author: Jorhabib Eljaik Gomez
% Istituto Italiano di Tecnologia
% Department of Robotics, Brain and Cognitive Sciences - RBCS

if nargin<2
    alpha=phi(1);
    upsilon=phi(2);
    psi=phi(3);

    R    =  [cos(alpha)*cos(upsilon)*cos(psi)-sin(alpha)*sin(psi)   -cos(alpha)*cos(upsilon)*sin(psi)-sin(alpha)*cos(psi)   cos(psi)*sin(upsilon);...
             sin(alpha)*cos(upsilon)*cos(psi)+cos(alpha)*sin(psi)   -sin(alpha)*cos(upsilon)*sin(psi)+cos(alpha)*cos(psi)   sin(alpha)*sin(upsilon);...
                       -sin(upsilon)*cos(psi)                                   sin(upsilon)*sin(psi)                       cos(upsilon)];
else
    if nargin<3
        Phi_cell = num2cell(phi,2);
        R        = cellfun(@euler2dcm, Phi_cell, 'UniformOutput', false);
    end

end

