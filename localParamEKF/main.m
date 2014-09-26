
% Department of Robotics, Barin and Cognitive Sciences
% Istituto Italiano di Tecnologia, 11 September 2014
%
% Original code by: Francesco Nori, 
% Modified to include local parametrization by: Jorhabib Eljaik G. 
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
% T_phi  : Transformation matrix between omega and dphi

clear
close all
clc

utilities    = genpath('./utils');
symb         = genpath('./symbolic');
addpath(utilities, symb)

%% Measurement model and it's derivative
f_func     = @forwardDynamics;
b_func     = @backwardDynamics;
h_func     = @rigidBodyOutput;
df_dx_func = @derivativeForwardDynamics;
dh_dx_func = @outputDerivatives;
db_dx_func = @derivativeBackwardDynamics;

%%
dt      = 0.01;      % sampling time
T       = 2.5;       % time span
sigma_f = 0.1;       % output error
sigma_u = 0.05;      % output error
sigma_a = 0.1;       % output error
n       = 21;        % state dimension (including additional force/torque)
m       = 9;         % output dimension

model.I   = diag([0.05 0.02 0.03]);
model.m   = 10;
model.u   = 0.5;
model.v   = 0.5;
model.ud  = 2.5;
model.vd  = 2.5;
model.th_init = pi/4;
R0        = [cos(model.th_init) -sin(model.th_init) 0; sin(model.th_init) cos(model.th_init) 0; 0 0 1];
model.x0  = [0*ones(6,1); 0*ones(12,1); dcm2euler(R0)];
model.dt  = dt;
model.g   = 9.81;
model.bck = false;

%% Inverse Dynamics - Planning
% This section computes the external force and torque datasets used in
% order to have a desired orientation away from singularities. 
useInvDyn = 'y';
plots = 0;
[f_B1_tid, mu_B1_tid, f_B2_tid, mu_B2_tid, t_invDyn] = InverseDynamics(T,model, plots);

%% Forward Dynamics
if(useInvDyn == 'y')
    model.x0 = [0*ones(6,1); dcm2euler(R0)];
end
[t,   x] = integrateForwardDynamics(model.x0, model, 0:dt:T, t_invDyn, f_B1_tid, mu_B1_tid, f_B2_tid, mu_B2_tid, useInvDyn);

% Let's compute the output used in the Kalman filter. In this specific
% case we will use dv^B, f^B and mu^B. In practice:
%
% y = [dv^B, f^B_1, f^B_2, mu^B_1, mu^B_2];


for i = 1:length(t)
    y(:,i) = rigidBodyOutput(x(i,:)',model);
end
y = y';

R         = diag([sigma_a.*ones(1,3), sigma_f.*ones(1,3), sigma_f.*ones(1,3), sigma_u.*ones(1,3), sigma_u.*ones(1,3)]);
y         = y + rand(size(y))*chol(R);

% Let's define the state of the Kalman filter. In this case we use an
% augmented state that includes also the applied forces and torques f^B and
% mu^B. Therefore we have:
%
% X = [dv^B, domega^b, f^B, mu^B, phi] = [x, f^B, mu^B, phi];
%
% Notice that since we have:
%
% dX = f(x) + Af*[f^B, mu^B]
% Y  = [f(x) + Af*[f^B, mu^B]](1:3)
%       f^B, mu^B];
%
% its discretization is:
%
% Xn     = X + df/dx(x)*dt + Af*[f^B, mu^B]*dt
% dh/dX  = [df/dx(1:3) Af(1:3)
%          0           I]


Xhat             = zeros(size(x));
% Q              = diag([ones(6,1).*dt*10000; ones(6,1)*10000; ones(4,1).*dt*10000;]);
a_Q  = 0.1;
f_Q  = 1;
mu_Q = 1; 
Q                = diag([a_Q*ones(3,1); f_Q*ones(6,1); mu_Q*ones(6,1)]);

% This is necessary to pretend that there's no torque applied to the system
% during each iteration of Kalman estimates, because this is modeled by the
% noise represented in Q. Kalman Filter does not assume external control
% input during and update time step [k -> k+1]

 model.u   = 0;
 model.v   = 0;
 model.ud  = 0;
 model.vd  = 0;
 
xh        = rand(size(model.x0)).*20 - 10;
Ph        = eye(n)*10000; 


for i = 1:length(t)
    % Update step
    % [xe, Pe, e, Lambda] = updateStepKF(xn', y(i-1,:)', C, Pn, R, model);
    [xh, Ph] = ekf_update1(xh , Ph, y(i,:)', dh_dx_func, R, h_func, [], model);

    Xupdt(i,:) = xh;
    
    % Prediction step update
    % [xn, Pn] = predictStepKF(Xhat(i-1,:)', P(:,:,i-1),          A, Q, model);
    [xh, Ph] =  ekf_predict1(xh, Ph, df_dx_func, Q, f_func, [], model);
     
    Xhat(i,:) = xh;
    P(:,:,i)  = Ph;
end

plotResults(x, Xupdt, P, t, 0)
plotResults(x, Xhat, P, t, 0)

%Smoother
% [Xhats,Ps] = etf_smooth1(Xhat',P,y',db_dx_func,Q,b_func,[],model, dh_dx_func,R,h_func,[],model, 1, 1);
% [M,P] =        ETF_SMOOTH1(M,P,Y,          A,Q,ia,W,aparam,H,R,h,V,hparam,same_p_a,same_p_h)
% plotResults(x, Xhats', Ps, t, 4)

