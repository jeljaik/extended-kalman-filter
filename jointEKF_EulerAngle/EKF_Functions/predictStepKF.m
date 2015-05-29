function [xn, Pn] = predictStepKF(xe, Pe, A, Q, model)

model.u  = model.u*0;
model.v  = model.v*0;

M               = zeros(12,12);
M(1:3  ,  1:3)  = eye(3).*model.m;
M(4:6  ,  4:6)  = model.I;
M(7:12 ,  7:12) = eye(6);
M(13:16, 13:16) = eye(4);

[ ~,x ] = integrateForwardDynamics( [0 model.dt], xe, model );
xn = x(end,:)';           % x[n+1|n]

%f =  rigidBodyDifferentialEquationImplicit(0, xe, model);
%xn = xe + M\f * model.dt;
Pn = A*Pe*A' + Q;      % P[n+1|n]
end