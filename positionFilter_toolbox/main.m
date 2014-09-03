
% Sestri Leveante, 23 Luglio 2014
%
% This piece of code simulates the problem of estimating dynamic quantities
% for a single rigid body with distributed force/torque measurements and
% distributed gyro/accelerometers measurements. The motion is governed by
% the following differential equation:
%
% m    dv^B    + S(omega^B) (m       v^B) = f^B_1  + ... + f^B_n + mg^B
%
% I^B domega^B + S(omega^B) (I^B omega^B) = mu^B_1 + ... + mu^B_n
%
%                                 dq      = 1/2 q_omega_B . q
%
% where we defined the following quantities:
%
% I^B    : inertia in the body reference frame
% m      : mass of the rigid body
% f^B_i  : i-th force expressed in the body reference frame
% mu^B_i : i-th torque expressed in the body reference frame
% omega^B: angular velocity expressed in the body reference frame
% v^B    : linear velocity in the body reference frame
% q      : quaternion representing the rigid body orientation

clear all
close all
clc

% Measurement model and it's derivative
f_func     = @forwardDynamics;
b_func     = @backwardDynamics;
h_func     = @rigidBodyOutput;
df_dx_func = @derivativeForwardDynamics;
dh_dx_func = @outputDerivatives;
db_dx_func = @derivativeBackwardDynamics;

dt      = 0.01;      % sampling time
T       = 5.0;       % time span
sigma_f = 0.1;       % output error
sigma_u = 0.05;      % output error
sigma_a = 0.1;       % output error
n       = 16;        % state dimension
m       = 9;         % output dimension

model.I   = diag([0.05 0.02 0.03]);
model.m   = 20;
model.u   = 10;
model.v   = 1;
R0        = eye(3);
model.x0  = [ones(6,1); ones(6,1); dcm2q(R0)'];
model.dt  = dt;
model.g   = 9.81;
model.bck = false;

% [tv1, f1]=ode45(@(t, x) rigidBodyDifferentialEquation(t, x, model),[0 5], model.x0, []);
% odeSettings = odeset('Mass', @(t,y)massMatrix(t,y,model), 'MStateDependence', 'strong');
% [tv3, f3]=ode45(@(t, x) rigidBodyDifferentialEquationImplicit(t, x, model),[0 5], model.x0, odeSettings);


[t,   x] = integrateForwardDynamics(model.x0, model, 0:dt:T);
% [tb, xb] = integrateBackwardDynamics(x(end,:), model, T:dt:2*T);

% Let's compute the output used in the Kalman filter. In this specific
% case we will use dv^B, f^B and mu^B. In practice:
%
% y = [dv^B, f^B, mu^B];
%

for i = 1:length(t)
    y(:,i) = rigidBodyOutput(x(i,:)',model);
end
y = y';
n
R         = diag([sigma_a.*ones(1,3), sigma_f.*ones(1,3), sigma_u.*ones(1,3)]);
y         = y + rand(size(y))*chol(R);

% Let's define the state of the Kalman filter. In this case we use an
% augmented state that includes also the applied forces and torques f^B and
% mu^B. Therefore we have:
%
% X = [dv^B, domega^b, f^B, mu^B, q] = [x, f^B, mu^B, q];
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
Q                = diag([ones(6,1); ones(6,1); ones(4,1);])*0.1;

model.u   = 0;
model.v   = 0;
xh        = rand(size(model.x0)).*20 - 10;
xh(13:16) = xh(13:16)/norm(xh(13:16));
Ph        = eye(n)*10000; 


for i = 1:length(t)
    % Update step
    % [xe, Pe, e, Lambda] = updateStepKF(xn', y(i-1,:)', C, Pn, R, model);
    [xh, Ph] = ekf_update1(xh , Ph, y(i,:)', dh_dx_func, R, h_func, [], model);

    % Prediction step update
    % [xn, Pn] = predictStepKF(Xhat(i-1,:)', P(:,:,i-1),          A, Q, model);
    [xh, Ph] =  ekf_predict1(xh, Ph, df_dx_func, Q, f_func, [], model);
    
    
    Xhat(i,:) = xh;
    P(:,:,i)  = Ph;
end

plotResults(x, Xhat, P, t, 0)

%Smoother
[Xhats,Ps] = etf_smooth1(Xhat',P,y',db_dx_func,Q,b_func,[],model, dh_dx_func,R,h_func,[],model, 1, 1);
% [M,P] =        ETF_SMOOTH1(M,P,Y,          A,Q,ia,W,aparam,H,R,h,V,hparam,same_p_a,same_p_h)
plotResults(x, Xhats', Ps, t, 4)

