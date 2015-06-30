function [dz_dq] = der_acc_model(x, param)
% Jacobian of the accelerometer measurement model for EKF
% Partial derivative w.r.t. q of: 
%
% acc_k = R*g_0 = q_k^{-1} [x] g_0 [x] q_k

if (~exist('quaternion','class'))
    error('Please add the utils directory to the path');
end

dt = param{1};
g0 = param{2};
qk = x;

derivative = 'jia'; %'jia', 'ligorio', 'r_qk'
%% The following uses the derivative of  the Quaternion Rotation Operator
% The derivative of the rotated 3-dimentional vector g0 w.r.t. the
% rotation is given  by the following expression. Reference: Yan-Bin Jian,
% Quaternions and Rotations. Notes.
q_real = qk(1);
q_vec = qk(2:4);
dz_dq = 2*[q_real*g0  + S(q_vec)*g0,  -g0*q_vec' + (dot(g0,q_vec))*eye(3) + q_vec*g0' - q_real*S(g0)]; 
dz_dq = [zeros(1,4); dz_dq];
% disp('Derivative Jia2015');
% disp(dz_dq);

%% TODO. This derivative is conceptually wrong. Devise the right expression for my representation of quaternions
% disp('Derivative Ligorio2013');
dz_dq_1 = S_R(qk)*S_R([0;g0])' + S_L(qk)'*S_L([0;g0])'*[-eye(3)   zeros(3,1); zeros(1,3)    1];
% disp(dz_dq_1);

%% Taking the derivative of R(qk) 
[Q0, Q1, Q2, Q3] = dQqdq(qk);
dR_dq_2 = [Q0*g0, Q1*g0, Q2*g0, Q3*g0];
% disp('Derivative of R(q)');
dR_dq_2 = [zeros(1,4); dR_dq_2];
% disp(dR_dq_2);

%% ################ Derivative of (R(qk))^-1 ###################
% RIGHT WAY TO DO IT!!
[Q0, Q1, Q2, Q3] = dQqdq(qk);
dR_dq_2 = [Q0'*g0, Q1'*g0, Q2'*g0, Q3'*g0];
% disp('Derivative of R(q)^-1');
dR_dq_2 = [zeros(1,4); dR_dq_2];
dz_dq = dR_dq_2;
% disp(dR_dq_2);

%% Assuming g_0 constant.
