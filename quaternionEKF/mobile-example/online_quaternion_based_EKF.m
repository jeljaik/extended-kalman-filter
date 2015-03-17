%
%
% To check consistency of the derivative, consider using DER_CHECK as done
% in EKFS_BOT_DEMO
%
% See also:
% MAINVISUALIZER DER_CHECK EKFS_BOT_DEMO

% EKF Parameters will be defined according to the nomenclature used in the
% function ekf_predict1 such that:
%
% M  - Nx1 mean state estimate after prediction step
% P  - NxN state covariance after prediction step
% Y  - Dx1 measurement vector.
% H  - Derivative of h() with respect to state as matrix,
%      inline function, function handle orl name
%      of function in form H(x,param)
% R  - Measurement noise covariance.
% h  - Mean prediction (innovation) as vector,
%      inline function, function handle or name
%      of function in form h(x,param).               (optional, default H(x)*X)
% V  - Derivative of h() with respect to noise as matrix,
%      inline function, function handle or name
%      of function in form V(x,param).               (optional, default identity)
% param - Parameters of h                            (optional, default empty)

function [MM, PP] = online_quaternion_based_EKF(interpOrientation, interpAccel, interpAngVel, dt, M, P, Qgyro, R, MM, PP, param, idx)

%% FUNCTIONS
% Handles to measurement model and its derivative
h_fun     = @acc_model;
dh_dx_fun = @der_acc_model;

%% EKF PARAMETERS
% The state in this demo consists on the 4-dim quaternion only ([q_real q_vec]
stateDim = 4;

% The following mapping of the [azimuth, pitch, roll] readings from
% the phone sensor define the local orientation frame on the phone
% which has z (azimuth) pointing up, x to the right (-pitch) and y (roll) according
% to the right hand rule. Orientation is given in DEGREES.

%% PREPARING FOR THE EKF
% Transition Matrix
% For the prediction step of the EKF we need matrices M, P, A and Q.
% The prior on M can be the initial (if known) orientation of the object as
% already set before.

% Each column corresponds to a sample
Y = interpAccel';
U = interpAngVel';
% disp('ang. velo')

%% Estimates with the EKF

% The discrete transition and covariance matrices in this case are
% functions of the angular velocity and its noise.
M = M.double;
[A,Q] = analyticalAQ(M, U, Qgyro, dt);
% EKF Prediction Step
[M,P] = ekf_predict1(M,P,A,Q);
% Preparing measurement with zero padding
yk = [0; Y];
yk = quaternion(yk);
yk = yk.normalize;
yk_d = yk.double;
% EKF Update
[M,P] = ekf_update1(M,P,yk_d,dh_dx_fun,R,h_fun,[],param);
M  = quaternion(M);
% The estimate of the EKF is not necessarily normalized.
M  = M.normalize;
M  = M.double;
MM = M;
PP = P;

end