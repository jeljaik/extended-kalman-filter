function [zk] = acc_model(x, param)
% Non-linear Accelerometer measurement model for EKF
%
% acc_k = R*g_0 = q_k [x] g_0 [x] q_k^{-1}
%
% Assuming quaternions where the first element is a scalar and last 
% three elements its vector components. [x] is the quaternion product operator. 
%
% TODO: SHOULD THE OUTPUT BE A QUATERNION OR A THREEDIMENSIONAL VECTOR?
%

if (~exist('quaternion','class'))
    error('Please add the utils directory to the path');
end

dt = param{1};
g0 = param{2};
% TODO: The following line is wrong as the original intention was to conver
% the zero-padded gravity vector into a quaternion to later multiply 
g0_q = quaternion([0;g0]);
sigmaMsmt = param{3};
qk = quaternion(x);
zk = product( product(qk,g0_q), inverse(qk) );
zk = zk.double;

%% Using the hypothesis that I should take the inverse of the rotation matrix associated to the estimated quaternion
%% ################### RIGHT WAY TO DO IT! #######################
zk = Qq(qk.double)'*g0;
zk = [0; zk];

end