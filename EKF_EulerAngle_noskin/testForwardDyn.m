

%clear
%close all
%clc

utilities    = genpath('./utils');
symb         = genpath('./symbolic');
%mexbm        = genpath('./mexWBModel');
ellipses      = genpath('./ellipses');
dynFuncs     = genpath('./dynamicsFunctions');
plotFuncs   = genpath('./plotFunctions');
%matlab_c3D =  genpath('./c3d_analysis');
skinFuncs   = genpath('./skinFunctions');

%addpath(utilities, symb, mexbm, ellipses,dynFuncs,plotFuncs)
addpath(utilities, symb, ellipses,dynFuncs,plotFuncs,skinFuncs)

%% Measurement model and its derivative
f_func     = @forwardDynamics;
b_func     = @backwardDynamics;
%h_func     = 
df_dx_func = @derivativeForwardDynamics;
dh_dx_func = @outputDerivatives;
db_dx_func = @derivativeBackwardDynamics;
h_func = @(x,model)rigidBodyOutput(x,model, [],[],[],[]);

source = 2; % 1 : sim data, 2 : real-data

T       = 2;        % estimation time span
n       = 21;         % state dimension - (translational vel, rotational vel, w_o, w_c, RPY angle)
m       = 19;         % output dimension

%% Kalman Parameters
% RealSensor parameters

%dt      = 0.01;      % sampling time
realKalman.T       = 2;       % time span
realKalman.sigma_f = 0.5;       % output error variance (forces)
realKalman.sigma_u = 0.75;      % output error variance (torques)
realKalman.sigma_a = 0.5;       % output error variance (acceleration)
realKalman.sigma_omega = 1.00;
realKalman.sigma_skin = 0.75;

realKalman.a_Q  = 1.0;
realKalman.omega_Q  = 2.0;
realKalman.f_Q  = 1.5;
realKalman.mu_Q = 2.5; 
realKalman.phi_Q = 7.5;

%realKalman.P = 0.001*diag([10*ones(6,1); 400*ones(6,1); 10*ones(6,1);20*ones(3,1)]);
realKalman.P =0.51* diag([ones(3,1);100*ones(3,1); ones(6,1); ones(6,1); 25*ones(3,1)]);
%realKalman.P = realKalman.P- 1*ones(size(realKalman.P)) + 5*rand(size(realKalman.P));
%% SimSensor parameters

simKalman.sigma_f = 0.25;       % output error variance (forces)
simKalman.sigma_u = 0.025;      % output error variance (torques)
simKalman.sigma_a = 0.5;       % output error variance (acceleration)
simKalman.sigma_omega = 0.5;
simKalman.sigma_skin = 0.5;

simKalman.a_Q  = 0.1;
simKalman.f_Q  = 0.1;
simKalman.mu_Q = 0.1; 
simKalman.skin_f_mu_Q = 0.1;
simKalman.phi_Q = 0.1;
simKalman.P = diag([ones(6,1); ones(6,1); ones(6,1); ones(3,1)]);

%P = diag([simKalman.sigma_a.*ones(1,3),simKalman.sigma_f.*ones(1,3), simKalman.sigma_f.*ones(1,3), simKalman.sigma_u.*ones(1,3), simKalman.sigma_u.*ones(1,3)]);

%% Model Parameters
model.I   = diag([0.05 0.02 0.03]);
model.m   = 0.761;%7;
model.dtInvDyn = 0.0001;
model.dtForDyn = 0.001;
model.dtKalman = 0.01;%0.05;%0.0025;%0.01;
model.g   = 9.81;
model.bck = false;

t_min = 5.0;%61;%42;
t_max = 6.5;%8.25;%64;%43;


[yMeas,tMeas,model,RData] = realMeasurement_completeLegWithSkin(model.dtKalman,model,0,t_min,t_max);
T = tMeas(end);
tKalman = tMeas;
   
f_Bo_tid = @(t)interp1(tMeas,yMeas(:,7:9),t)';
mu_Bo_tid = @(t)interp1(tMeas,yMeas(:,10:12),t)';
f_Bc_tid = @(t)interp1(tMeas,yMeas(:,13:15),t)';
mu_Bc_tid = @(t)interp1(tMeas,yMeas(:,16:18),t)';

[tint,xint] = integrateForwardDynamics( model.x0, model, tMeas, f_Bo_tid, mu_Bo_tid, f_Bc_tid, mu_Bc_tid, 'lala');

figure;
subplot(3,3,1);
plot(tint,xint(:,19)); axis tight;
xlabel('t');
ylabel('\phi_x');
title('Orientation');
subplot(3,3,4);
plot(tint,xint(:,20)); axis tight;
xlabel('t');
ylabel('\phi_y');
subplot(3,3,7);
plot(tint,xint(:,21)); axis tight;
xlabel('t');
ylabel('\phi_z');


%figure;
subplot(3,3,2);
plot(tint,xint(:,1)); axis tight;
xlabel('t');
ylabel('v_x');
title('Linear velocity');
subplot(3,3,5);
plot(tint,xint(:,2)); axis tight;
xlabel('t');
ylabel('v_y');
subplot(3,3,8);
plot(tint,xint(:,3)); axis tight;
xlabel('t');
ylabel('v_z');


%figure;
subplot(3,3,3);
plot(tint,xint(:,4)); axis tight;
xlabel('t');
ylabel('\omega_x');
title('Angular velocity');
subplot(3,3,6);
plot(tint,xint(:,5)); axis tight;
xlabel('t');
ylabel('\omega_y');
subplot(3,3,9);
plot(tint,xint(:,6)); axis tight;
xlabel('t');
ylabel('\omega_z');
% 
% figure;
% %%acceleration assumption
% plot(tint,x(:,

y = zeros(length(tint),19);
%% Sensor comparisons
for tid = 1:length(tint)
    tT = tint(tid);
    y(tid,:) = rigidBodyOutput(xint(tid,:)',model,f_Bo_tid(tT),mu_Bo_tid(tT),f_Bc_tid(tT),mu_Bc_tid(tT));
end
t = tint;
figure;
subplot(2,1,1);
plot(t,y(:,1:3)); hold on;
plot(t,yMeas(:,1:3),'--'); axis tight;
ylabel('Acceleration m/sec^2');
xlabel('Time t(sec)');

subplot(2,1,2);
plot(t,(y(:,1:3) - yMeas(:,1:3)).^2.0); axis tight;
ylabel('squared acceleration error m/sec^2');
xlabel('Time t(sec)');

figure;
subplot(2,1,1);
plot(t,y(:,4:6));hold on;
plot(t,yMeas(:,4:6),'--'); axis tight;
ylabel('Angular velocity \omega rad/sec');
xlabel('Time t(sec)');

subplot(2,1,2);
plot(t,(y(:,4:6) - yMeas(:,4:6)).^2.0); axis tight;
ylabel('Squared angular velocity error \omega rads/sec');
xlabel('Time t(sec)');

