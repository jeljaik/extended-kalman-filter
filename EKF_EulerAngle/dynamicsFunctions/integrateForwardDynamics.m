function [ t,x ] = integrateForwardDynamics( x0, model, time_span, f_B1_tid, mu_B1_tid, f_B2_tid, mu_B2_tid, useInvDyn)

[n, ~]           = size(x0);
M                = zeros(n,n);
M(1:3   ,  1:3)  = eye(3).*model.m;
M(4:6   ,  4:6)  = model.I;
%if ~useInvDyn
    M(7:18  ,  7:18) = eye(12);      % Both external forces
    M(19:21 , 19:21) = eye(3);
%else
%    M(7:9 , 7:9) = eye(3);
%end
odeSettings      = odeset('Mass', M, 'InitialStep', 1e-12);%,'MaxStep',1e-5);
model.bck        = false;

% @(t)interp1(model.tc,[model.fc1,model.muc1,model.fc2,model.muc2]',t)

[t, x]=ode15s(@(t, x) rigidBodyDifferentialEquationImplicit(t, x, model,f_B1_tid, mu_B1_tid, f_B2_tid, mu_B2_tid, useInvDyn ), time_span, x0, odeSettings);

x=x;
end

