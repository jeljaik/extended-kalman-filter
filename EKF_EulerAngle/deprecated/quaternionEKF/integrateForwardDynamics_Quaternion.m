function [ t,x ] = integrateForwardDynamics_Quaternion( x0, model, time_span)

[n, ~]           = size(x0);
M                = zeros(n,n);
M(1:3   ,  1:3)  = eye(3).*model.m;
M(4:6   ,  4:6)  = model.I;
M(7:12  ,  7:12) = eye(6);
M(13:16 , 13:16) = eye(4);
odeSettings      = odeset('Mass', M);
model.bck        = false;

[t, x]=ode45(@(t, x) rigidBodyDifferentialEquationImplicit_Quaternion(t, x, model), time_span, x0, odeSettings);


end

