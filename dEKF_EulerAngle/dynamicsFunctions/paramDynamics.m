function wn = forwardDynamics( x0, model)

% [n, ~]           = size(x0);
% M                = zeros(n,n);
% M(1:3   ,  1:3)  = eye(3).*model.m;
% M(4:6   ,  4:6)  = model.I;
% M(7:18  ,  7:18) = eye(12);      % Both external forces
% M(19:21 , 19:21) = eye(3);
% 
% odeSettings      = odeset('Mass', M, 'InitialStep', 1e-16);%, 'MaxStep', 1e-6);
dt               = model.dtKalman;
% 
% [~, x]=ode15s(@(t, x) rigidBodyDifferentialEquationImplicit...
% (t, x, model, [0 dt], zeros(2,3),zeros(3,2), zeros(3,2), zeros(3,2), 'y'), [0 dt], x0, odeSettings)

%xn = x(end,:)';

initParam = @(t,x) paramDifferentialEquation(t, x0, model,...
    [], [], [], [], 'y');
%[~, x]=ode15s(@(t, x) rigidBodyDifferentialEquationImplicit(t, x, model),
%[0 dt], x0, odeSettings);
[~,w] = ode15s(initParam,[0 model.dtKalman], model.w0);
wn = w(end,:)';

%end




end

