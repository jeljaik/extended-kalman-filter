function xn = forwardDynamics_dualState( x0, model)

[n, ~]           = size(x0);
M                = zeros(n,n);
M(1:3   ,  1:3)  = eye(3).*model.m;
M(4:6   ,  4:6)  = model.I;
M(7:18  ,  7:18) = eye(12);      % Both external forces
M(19:21 , 19:21) = eye(3);
M(22:30   ,  22:30)  = eye(9);
odeSettings      = odeset('Mass', M, 'InitialStep', 1e-10);%, 'MaxStep', 1e-6);

zeroForcedProcess = @(t,x) processImplicitODE_dualState(t, x, model,...
    [], [], [], [], 'y');
[~,x] = ode15s(zeroForcedProcess,[0 model.dtKalman], x0, odeSettings);
xn = x(end,:)';




end

