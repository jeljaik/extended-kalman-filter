
% Department of Robotics, Barin and Cognitive Sciences
% Istituto Italiano di Tecnologia, 11 September 2014
%
% Original code by: Francesco Nori, 
% Modified to include local parametrization by: Jorhabib Eljaik G. and
% Naveen Kuppuswamy
% 
% This piece of code simulates the problem of estimating dynamic quantities
% for a single rigid body with distributed force/torque measurements and
% distributed gyro/accelerometers measurements. The motion is governed by
% the following differential equation with a local parametrization of the
% orientation in ZYZ Euler angles.
%
% m    dv^B    + S(omega^B) (m       v^B) = f^B_1  + ... + f^B_n + mg^B
%
% I^B domega^B + S(omega^B) (I^B omega^B) = mu^B_1 + ... + mu^B_n
%
%                                  dphi   = T_phi^-1 * omega
%
% where we defined the following quantities:
%
% I^B    : inertia in the body reference frame
% m      : mass of the rigid body
% f^B_i  : i-th force expressed in the body reference frame
% mu^B_i : i-th torque expressed in the body reference frame
% omega^B: angular velocity expressed in the body reference frame
% v^B    : linear velocity in the body reference frame
% phi    : ZYZ Euler angles representing orientation.
% T_phi  : Transformation matrix between omeg%diff(torsoVelDash)./repmat(diff(t(1:end-1),1,1),1,3);


%clear
close all
clc

utilities    = genpath('./utils');
symb         = genpath('./symbolic');
addpath(utilities, symb)

%% Measurement model and its derivative
f_func     = @forwardDynamics;
b_func     = @backwardDynamics;
%h_func     = 
df_dx_func = @derivativeForwardDynamics;
dh_dx_func = @outputDerivatives;
db_dx_func = @derivativeBackwardDynamics;
h_func = @(x,model)rigidBodyOutput(x,model, [],[],[],[]);

source = 2; % 1 : sim data, 2 : real-data

%% Kalman Parameters
%dt      = 0.01;      % sampling time
T       = 1.5   ;       % time span
sigma_f = 0.025;       % output error variance (forces)
sigma_u = 0.025;      % output error variance (torques)
sigma_a = 0.01;       % output error variance (acceleration)
sigma_omega = 0.01;
n       = 21;%21;      % statedimension - (translational vel, rotational vel, RPY angle)  % older : state dimension (including additional force/torque)
m       = 12;         % output dimension

%% Model Parameters
model.I   = diag([0.05 0.02 0.03]);
model.m   = 7;
model.dtInvDyn = 0.0001;
model.dtForDyn = 0.001;
model.dtKalman = 0.01;%0.01;
model.g   = 9.81;
model.bck = false;

tKalman = 0:model.dtKalman:T;

R         = diag([sigma_a.*ones(1,3),sigma_f.*ones(1,3), sigma_f.*ones(1,3), sigma_u.*ones(1,3), sigma_u.*ones(1,3)]);

%% chose source of data (simulation or real-robot)
if(source ==1) 
    [yMeas,model] = simulatedMeasurement(tKalman,R,model,'forceSim',1); % set the last parameter to empty to use saved simulation data if exists
    
else
    [yMeas,tMeas,model] = realMeasurement(model.dtKalman,model,1);
    T = tMeas(end);
    tKalman = tMeas;
end

% Q              = diag([ones(6,1).*dt*10000; ones(6,1)*10000; ones(4,1).*dt*10000;]);
a_Q  = 0.001;
f_Q  = 0.04;
mu_Q = 0.04; 
phi_Q = 0.001;
%Q                = diag([a_Q*ones(3,1); f_Q*ones(6,1); mu_Q*ones(6,1)]);
Q  = diag([a_Q*ones(3,1); f_Q*ones(6,1); mu_Q*ones(6,1); phi_Q*ones(3,1)]);


 xh        = model.x0;% + 0.1*randn(size(model.x0));
%xh        = rand(s0(1)+12,s0(2)).*20 - 10;
Ph        = 0.0001*diag([20;20*ones(5,1); 15*ones(6,1); 15*ones(6,1);15*ones(3,1)]); 

% updating 100 times faster than reality
%model.dt = model.dt/100;
%t = 

Xhat             = zeros(n,length(tKalman))';
%% Faking a measured signal from the output of interated forward dynamics

model.dt = model.dtKalman;


Xupdt = zeros(length(tKalman),n);
P = zeros(size(Ph,1), size(Ph,2),length(tKalman));


disp('Starting Kalman Filter prediction');
drawnow;

for i = 1:length(tKalman)
    % Update step
    % [xe, Pe, e, Lambda] = updateStepKF(xn', y(i-1,:)', C, Pn, R, model);
    %[xh, Ph] = ekf_update1(xh , Ph, y(i,:)', dh_dx_func, R, h_func, [], model);
    [xh, Ph] = ekf_update1(xh , Ph, yMeas(i,:)', dh_dx_func, R,...
        h_func, [], model);

    Xupdt(i,:) = xh;
    
    % Prediction step update
    % [xn, Pn] = predictStepKF(Xhat(i-1,:)', P(:,:,i-1),          A, Q, model);
    [xh, Ph] =  ekf_predict1(xh, Ph, df_dx_func, Q, f_func, [], model);
     
    Xhat(i,:) = xh;
    P(:,:,i)  = Ph;
end


%plotResults(xForDyn, tForDyn, Xupdt, P, tKalman,  0)
plotResultsOutput(Xupdt, P, tKalman, yMeas);
%plotResults(xForDyn, tForDyn, Xhat, P, tKalman, 0)

%Smoother
% [Xhats,Ps] = etf_smooth1(Xhat',P,y',db_dx_func,Q,b_func,[],model, dh_dx_func,R,h_func,[],model, 1, 1);
% [M,P] =        ETF_SMOOTH1(M,P,Y,          A,Q,ia,W,aparam,H,R,h,V,hparam,same_p_a,same_p_h)
% plotResults(x, Xhats', Ps, t, 4)

