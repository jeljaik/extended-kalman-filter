function [ T ] = Tomega_dphi( phi, t )
% TOMEGA_DPHI(PHI) Returns the transformation matrix used to convert the
% time derivative of Phi into angular velocity omega.
%
% TOMEGA_DPHI(PHI,'T') Same as TOMEGA_DPHI(PHI), except that it recieves as
% input a time varying Phi. Returns T as a cell array of same length of Phi
% e.g T   = Tomega_dphi(phi,'t');
%     T_1 = T{1}; 

if nargin < 2 
    alpha   = phi(1);
    upsilon = phi(2);
    psi     = phi(3);

    T = [0  -sin(alpha) cos(alpha)*sin(upsilon);...
         0   cos(alpha) sin(alpha)*sin(upsilon);...
         1       0      cos(upsilon)];
else
    if nargin < 3
        Phi_cell = num2cell(phi,2);
        T = cellfun(@Tomega_dphi, Phi_cell, 'UniformOutput', false);
    end


end

