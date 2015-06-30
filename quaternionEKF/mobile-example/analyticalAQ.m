% Analytical equations to retrieve the discretized transition and
% covariance matrices.

function [A,Q] = analyticalAQ(qk, wk, SigmaGyro, dt)

if size(qk,1)~=4 || size(qk,2)~=1
    error('Input quaternion must be of size 4x1 where first element is the real part');
end
if size(wk,1) ~= 3 || size(wk,2) ~= 1
    error('Input angular velocity must be of size (3x1)');
end
if size(SigmaGyro,1) ~= 3 || size(SigmaGyro,2) ~= 3
    error('Process covariance matrix must be of size (3x3)');
end
    
%% Notes on Omega and Xi
% The following expressions for Xi and Omega are different from the ones in
% Trawny because he considers quaternions where the last element is the
% real part, while the representation used in this project has the real
% part of the quaternion its first element.
Omega = [0    -wk'; 
        wk   -S(wk)];

Xi = [      -qk(2:4,:)';
      qk(1)*eye(3) + S(qk(2:4,:)) ];
      
A = eye(4) + 0.5*Omega*dt;
Q = (dt/2)^2*Xi*SigmaGyro*Xi';
end