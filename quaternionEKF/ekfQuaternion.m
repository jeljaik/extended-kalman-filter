%% EXTENDED KALMAN FILTER FOR QUATERNIONS
clear all
close all
clc

%% Setup
% Integration/sampling time
dt = 0.010;
% Time horizon
T  = 10;
% Time
t = 0:dt:T;

%% Model
gyroModel.sigma_r = 0.1; % rate noise covariance (measurement noise)
gyroModel.sigma_w = 0.1; % gyro noise covariance
gyroModel.dt       = dt;  % integration time.
gyroModel.scale_omega_m = 10;

%% Initial conditions
q_hat_k =       [zeros(3,1); 1];
b_hat_k =       zeros(3,1);
omega_hat_k =   zeros(3,1);
omega_m_1 =     zeros(3,1);
P_k =           eye(6)*10000;
eps =           0.001;

%% Gyro measurements over time
omega_m = [sin(t)*gyroModel.scale_omega_m;...
           zeros(1,length(t));...
           zeros(1,length(t))];

%% Estimation 
history.q_bar_hat_1 = zeros(4,length(t));
history.b_hat_1     = zeros(3,length(t));
history.omega_hat_1 = zeros(3,length(t));
for k=1:length(t)
    omega_m_1 = omega_m(:,k);
    [P_1, q_bar_hat_1, b_hat_1, omega_hat_1] = ekfqEstimate(gyroModel, q_hat_k, b_hat_k, omega_hat_k, omega_m_1, P_k, eps);
    
    history.q_bar_hat_1(:,k)=q_bar_hat_1;
    history.b_hat_1(:,k)=b_hat_1;
    history.omega_hat_1(:,k)=omega_hat_1;
    
    P_1 = P_k;
    q_hat_k = q_bar_hat_1;
    b_hat_k = b_hat_1;
    omega_hat_k = omega_hat_1;
end


%% Update

