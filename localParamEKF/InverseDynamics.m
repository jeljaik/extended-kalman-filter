function [f_B1_t, mu_B1_t, f_B2_t, mu_B2_t, tc] = InverseDynamics(T, model, plots)
% This code computes  inverse dynamics for a rigid body system assuming null 
% linear velocity and a desired orientation trajectory parametrized in ZYZ 
% Euler angles (roll, pitch and yaw).
% Author: Jorhabib Eljaik
% Istituto Italiano di Tecnologia
% Department of Robotics, Brain and Cognitive Sciences (RBCS)

%% Desired orientation trajectory
tc = 0:model.dt:T;

alpha   = pi/4*cos(tc)'; 
upsilon = pi/4*ones(length(tc),1); 
psi     = zeros(length(tc),1);
Phi     = [    alpha      upsilon    psi     ]';

%% Required external wrenches
[f_B1_t, mu_B1_t, f_B2_t, mu_B2_t] = rigidBodyInvDyn(Phi, model, 0.5, 0.5);

%% Plots
if plots
    plot(tc(1:end-2),f_B1_t, 'r'); hold on;
    plot(tc(1:end-2),f_B2_t, '.b');
end

tc = tc(1:end-2); %% After numerical derivatives

end