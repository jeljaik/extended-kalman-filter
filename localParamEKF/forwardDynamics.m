function xn = forwardDynamics( x0, model)

[n, ~]           = size(x0);
M                = zeros(n,n);
M(1:3   ,  1:3)  = eye(3).*model.m;
M(4:6   ,  4:6)  = model.I;
M(7:12  ,  7:12) = eye(6);
M(13:15 , 13:15) = eye(3);
odeSettings      = odeset('Mass', M);
dt               = model.dt;

[~, x]=ode45(@(t, x) rigidBodyDifferentialEquationImplicit(t, x, model), [0 dt], x0, odeSettings);
xn = x(end,:)';

end

