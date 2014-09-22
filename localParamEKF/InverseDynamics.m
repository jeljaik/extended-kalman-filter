% This code computes  inverse dynamics for a rigid body system assuming null 
% linear velocity and a desired orientation trajectory parametrized in ZYZ 
% Euler angles (roll, pitch and yaw).
% Author: Jorhabib Eljaik
% Istituto Italiano di Tecnologia
% Department of Robotics, Brain and Cognitive Sciences (RBCS)

%% Simulation parameters and model information

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
th = pi/4;
R0        = [cos(th) -sin(th) 0; sin(th) cos(th) 0; 0 0 1];
model.x0  = [0*ones(6,1); 0*ones(12,1); dcm2euler(R0)];
model.dt  = dt;
model.g   = 9.81;
model.bck = false;

%% Desired orientation trajectory
t = 0:dt:T;

alpha   = pi/4*cos(t)'; 
upsilon = pi/4*ones(length(t),1); 
psi     = zeros(length(t),1);
Phi     = [    alpha      upsilon    psi     ]';

%% Required external wrenches
[f_B1_t, mu_B1_t, f_B2_t, mu_B2_t] = rigidBodyInvDyn(Phi, model, 0.5, 0.5);
plot(t(1:end-2),f_B1_t, 'r'); hold on;
plot(t(1:end-2),f_B2_t, '.b');
