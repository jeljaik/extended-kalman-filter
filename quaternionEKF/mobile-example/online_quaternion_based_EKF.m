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

function [MM, PP] = online_quaternion_based_EKF(interpOrientation, interpAccel, interpAngVel, dt, MM, PP, Qgyro, R, param, idx)

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
[A,Q] = analyticalAQ(MM, U, Qgyro, dt);
% EKF Prediction Step
[MM,PP] = ekf_predict1(MM,PP,A,Q);
% Preparing measurement with zero padding
yk = [0; Y];
yk = quaternion(yk);
yk = yk.normalize;
yk_d = yk.double;
% Update Measurement Noise Covariance Matrix R
k0 = 0.0001; k1 = 0.001 ; k2 = 0.0001;
Rvar = k0 + k1*(sqrt(U(1)^2 + U(2)^2 + U(3)^2)) + k2*abs(9.81 - norm(Y));
R = Rvar*eye(3);
R = [[Rvar 0 0 0]; [0 0 0]' R];
% EKF Update
[MM,PP] = ekf_update1(MM,PP,yk_d,dh_dx_fun,R,h_fun,[],param);
MM  = quaternion(MM);
% The estimate of the EKF is not necessarily normalized.
MM  = MM.normalize;
MM  = MM.double;

end