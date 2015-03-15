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
M = quaternion.eulerangles('xyz', pi/180*[-interpOrientation(1,2), interpOrientation(1,3), interpOrientation(1,1)]);
M = M.double;

% Process (state) covariance matrix. Small because we're certain about the
% initial orientation on all components of the quaternion.
procCov = 0.01;
Qc = procCov*eye(stateDim);

% Gyro covariance matrix
stdGyro = 0.1;
Qgyro = stdGyro*eye(3);

% Measurement noise covariance
measCov = 0.1;
R = measCov*eye(4);

%% PREPARING FOR THE EKF
% Transition Matrix
% For the prediction step of the EKF we need matrices M, P, A and Q. 
% The prior on M can be the initial (if known) orientation of the object as
% already set before.
using_lti_disc = 0;
param{1} = dt;
% Param{2} is gravity in g units. This is convenient for the quaternion
% representation. MEASUREMENTS SHOULD BE NORMALIZED THEN.
param{2} = [0 0 1]';
param{3} = measCov;

% Each column corresponds to a sample
Y = interpAccel';
U = interpAngVel';
totSamples = size(Y,2);

%% Checking derivatives
der_check(h_fun, dh_dx_fun, 1, M, param);

%% Estimates with the EKF
% Allocating space for estimates
P  = 1*eye(4);
MM = zeros(size(M,1), totSamples);
PP = zeros(size(M,1), size(M,1), totSamples);
disp('Performing batch predictions...');
for k=1:totSamples
    % The discrete transition and covariance matrices in this case are
    % functions of the angular velocity and process noise. 
    if (using_lti_disc)
        % I know what F is like, but I'm not entirely sure of what L should
        % be like!! How should the gyro noise be passed?
        F = [0        -U(:,k)'; 
             U(:,k)   -S(U(:,k)) ];
        L = eye(4);
        [A,Q] = lti_disc(F,L,Qc,dt);
    else
        [A,Q] = analyticalAQ(M, U(:,k), Qgyro, dt);
    end
    % EKF Prediction Step
    [M,P] = ekf_predict1(M,P,A,Q);    
    % EKF Update Step
%     yk = quaternion.eulerangles('xyz',Y(:,k));
%     yk_d = yk.double;
    % Measurement in Euler Angles in case measurement model and its
    % derivative is expressed in terms of the 
    yk = [0; Y(:,k)];
    yk = quaternion(yk);
    yk = yk.normalize;
    yk_d = yk.double;
    [M,P] = ekf_update1(M,P,yk_d,dh_dx_fun,R,h_fun,[],param);
    M = quaternion(M);
    % The estimate of the EKF is not necessarily normalized.
    M = M.normalize;
    M = M.double;
    MM(:,k) = M;
    PP(:,:,k) = P; 
end

%% Plot prediction results
% In the next line I transform all the real orientations as given by the
% IMU of the cellphone into quaternions. 
% The following mapping of the [azimuth, pitch, roll] readings from
% the phone sensor define the local orientation frame on the phone
% which has z (azimuth) pointing up, x to the right (-pitch) and y (roll) according
% to the right hand rule. Orientation is given in DEGREES. 
interpOrientation_quat = quaternion.eulerangles('xyz', pi/180*[-interpOrientation(:,2), interpOrientation(:,3), interpOrientation(:,1)]);
% interpOrientation_quat = quaternion([zeros(1,size(interpOrientation',2)); pi/180*interpOrientation']);
MM_quat = quaternion(MM);
plot_predictions(newTime, MM_quat, PP, interpOrientation_quat);
