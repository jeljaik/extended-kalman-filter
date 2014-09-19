function [ t,x ] = integrateForwardDynamics( x0, model, time_span)

[n, ~]           = size(x0);
M                = zeros(n,n);
M(1:3   ,  1:3)  = eye(3).*model.m;
M(4:6   ,  4:6)  = model.I;
M(7:18  ,  7:18) = eye(12);      % Both external forces
M(19:21 , 19:21) = eye(3);
odeSettings      = odeset('Mass', M, 'InitialStep', 1e-12);
model.bck        = false;

[t, x]=ode15s(@(t, x) rigidBodyDifferentialEquationImplicit(t, x, model), time_span, x0, odeSettings);


end

