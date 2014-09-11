function [ T ] = Tomega_dphi( phi )
%TOMEGA_DPHI Returns T from dphi = T*omega where 'omega' is angular
% velocity and 'dphi' is the time derivative of a ZYZ euler angle vector.

alpha   = phi(1);
upsilon = phi(2);
psi     = phi(3);

T = [0  -sin(alpha) cos(alpha)*sin(upsilon);...
     0   cos(alpha) sin(alpha)*sin(upsilon);...
     1       0      cos(upsilon)];


end

