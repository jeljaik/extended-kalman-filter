function xn = backwardDynamics( x0, model)

[n, ~]           = size(x0);
M                = zeros(n,n);
% M(1:3   ,  1:3)  = eye(3).*model.m;
% M(4:6   ,  4:6)  = model.I;
% M(7:12  ,  7:12) = eye(6);
% M(13:16 , 13:16) = - eye(4);
M(1:3   ,  1:3)  = eye(3).*model.m;
M(4:6   ,  4:6)  = model.I;
M(7:18  ,  7:18) = eye(12);      % Both external forces
M(19:21 , 19:21) = -eye(3);
odeSettings      = odeset('Mass', -M);
dt               = model.dtKalman;
model.bck        = true;

zeroForcedRigidBody = @(t,x) rigidBodyDifferentialEquationImplicit(t, x, model,...
    [], [], [], [], 'y');
%[~, x]=ode15s(@(t, x) rigidBodyDifferentialEquationImplicit(t, x, model), [0 dt], x0, odeSettings);

[~,x] = ode15s(zeroForcedRigidBody,[0 model.dtKalman], x0, odeSettings);
xn = x(end,:)';

end

