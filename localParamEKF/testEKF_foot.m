
% Department of Robotics, Brain and Cognitive Sciences
% Istituto Italiano di Tecnologia, 11 September 2014
%
% Original code by: Francesco Nori, 
% Modified to include local parametrization by: Jorhabib Eljaik G. and
% Naveen Kuppuswamy
% 
% This piece of code simulates the problem of estimating dynamic quantities
% for a single rigid body with distributed force/torque measurements and
% distributed gyro/accelerometers measurements. The motion is governed by
% the following differential equation with a local parametrization of the
% orientation in ZYZ Euler angles.
%
% m    dv^B    + S(omega^B) (m       v^B) = f^B_1  + ... + f^B_n + mg^B
%
% I^B domega^B + S(omega^B) (I^B omega^B) = mu^B_1 + ... + mu^B_n
%
%                                  dphi   = T_phi^-1 * omega
%
% where we defined the following quantities:
%
% I^B    : inertia in the body reference frame
% m      : mass of the rigid body
% f^B_i  : i-th force expressed in the body reference frame
% mu^B_i : i-th torque expressed in the body reference frame
% omega^B: angular velocity expressed in the body reference frame
% v^B    : linear velocity in the body reference frame
% phi    : ZYZ Euler angles representing orientation.
% T_phi  : Transformation matrix between omeg%diff(torsoVelDash)./repmat(diff(t(1:end-1),1,1),1,3);


%clear
close all
clc

utilities    = genpath('./utils');
symb         = genpath('./symbolic');
mexbm        = genpath('./mexWBModel');
ellipses      = genpath('./ellipses');
%matlab_c3D =  genpath('./c3d_analysis');

addpath(utilities, symb, mexbm, ellipses)

%% Measurement model and its derivative
f_func     = @forwardDynamics;
b_func     = @backwardDynamics;
%h_func     = 
df_dx_func = @derivativeForwardDynamics;
dh_dx_func = @outputDerivatives;
db_dx_func = @derivativeBackwardDynamics;
h_func = @(x,model)rigidBodyOutput(x,model, [],[],[],[]);

source = 1; % 1 : sim data, 2 : real-data

T       = 8.0;        % estimation time span
n       = 21;         % state dimension - (translational vel, rotational vel, w_o, w_c, RPY angle)
m       = 18;         % output dimension

%% Kalman Parameters
% RealSensor parameters

%dt      = 0.01;      % sampling time
realKalman.T       = 1.5;       % time span
realKalman.sigma_f = 0.5;       % output error variance (forces)
realKalman.sigma_u = 0.25;      % output error variance (torques)
realKalman.sigma_a = 0.5;       % output error variance (acceleration)
realKalman.sigma_omega = 0.05;


realKalman.a_Q  = 0.01;
realKalman.f_Q  = 0.04;
realKalman.mu_Q = 0.04; 
realKalman.phi_Q = 0.005;

realKalman.P = 0.001*diag([10*ones(6,1); 400*ones(6,1); 10*ones(6,1);20*ones(3,1)]);


%% SimSensor parameters

simKalman.sigma_f = 0.25;       % output error variance (forces)
simKalman.sigma_u = 0.025;      % output error variance (torques)
simKalman.sigma_a = 0.5;       % output error variance (acceleration)
simKalman.sigma_omega = 0.05;

simKalman.a_Q  = 0.001;
simKalman.f_Q  = 0.04;
simKalman.mu_Q = 0.04; 
simKalman.phi_Q = 0.001;
simKalman.P = 0.01*diag([50;10*ones(5,1); 1*ones(6,1); 15*ones(6,1);20*ones(3,1)]);

%P = diag([simKalman.sigma_a.*ones(1,3),simKalman.sigma_f.*ones(1,3), simKalman.sigma_f.*ones(1,3), simKalman.sigma_u.*ones(1,3), simKalman.sigma_u.*ones(1,3)]);

%% Model Parameters
model.I   = diag([0.05 0.02 0.03]);
model.m   = 7;
model.dtInvDyn = 0.0001;
model.dtForDyn = 0.001;
model.dtKalman = 0.01;%0.01;
model.g   = 9.81;
model.bck = false;

t_min = 61;%42;
t_max = 64;%43;



%% chose source of data (simulation or real-robot)
if(source ==1) 
    kalman = simKalman;
    R =diag([kalman.sigma_a.*ones(1,3),kalman.sigma_omega.*ones(1,3),kalman.sigma_f.*ones(1,3), kalman.sigma_f.*ones(1,3), kalman.sigma_u.*ones(1,3), kalman.sigma_u.*ones(1,3)]);
    tKalman = 0:model.dtKalman:T;
    [yMeas,model] = simulatedMeasurement(tKalman,R,model,'forceSim',1); % set the last parameter to empty to use saved simulation data if exists
    
    
else
    [yMeas,tMeas,model] = realMeasurement(model.dtKalman,model,0,t_min,t_max);
    T = tMeas(end);
   tKalman = tMeas;
   % numSamples = length(tKalman) - 4000;
   numJump = 1;%10;
    
  %  yMeas = yMeas(numSamples:end,:);tKalman = tKalman(numSamples:end);
   yMeas = yMeas(1:numJump:end,:);
   tKalman = tKalman(1:numJump:end);
   kalman = realKalman;
end
% remove next line later
%kalman.P = diag([kalman.sigma_a.*ones(1,3),kalman.sigma_f.*ones(1,3), kalman.sigma_f.*ones(1,3), kalman.sigma_u.*ones(1,3), kalman.sigma_u.*ones(1,3)]);


%% KALMAN FILTER IMPLEMENTATION
Q  = diag([kalman.a_Q*ones(3,1); 
           kalman.f_Q*ones(6,1); 
           kalman.mu_Q*ones(6,1); 
           kalman.phi_Q*ones(3,1)]);
R =diag([kalman.sigma_a.*ones(1,3), kalman.sigma_omega.*ones(1,3), kalman.sigma_f.*ones(1,3), kalman.sigma_f.*ones(1,3), kalman.sigma_u.*ones(1,3), kalman.sigma_u.*ones(1,3)]);
Ph = kalman.P;

% Initializing estimate
xh        = model.x0;% + 0.1*randn(size(model.x0));
Xhat      = zeros(n,length(tKalman))';

% Initializing update
model.dt = model.dtKalman;
Xupdt = zeros(length(tKalman),n);
P = zeros(size(Ph,1), size(Ph,2),length(tKalman));

disp('Starting Kalman Filter prediction');
drawnow;

for i = 1:length(tKalman)
    if(mod(i,10)==0)
        fprintf('Timenow : ');disp(tKalman(i)); drawnow();
    end
    tic;
    % Update step
    % [xe, Pe, e, Lambda] = updateStepKF(xn', y(i-1,:)', C, Pn, R, model);
    %[xh, Ph] = ekf_update1(xh , Ph, y(i,:)', dh_dx_func, R, h_func, [], model);
    [xh, Ph] = ekf_update1(xh , Ph, yMeas(i,:)', dh_dx_func, R,...
        h_func, [], model);

    Xupdt(i,:) = xh;
    
    % Prediction step update
    % [xn, Pn] = predictStepKF(Xhat(i-1,:)', P(:,:,i-1),          A, Q, model);
    [xh, Ph] =  ekf_predict1(xh, Ph, df_dx_func, Q, f_func, [], model);
     
    Xhat(i,:) = xh;
    P(:,:,i)  = Ph;
    if(mod(i,10)==0)
        fprintf('Step processing time :');
        disp(toc());
    end
    
end


%plotResults(xForDyn, tForDyn, Xupdt, P, tKalman,  0)
plotResultsOutput_noGyro(Xupdt, Xhat, P, tKalman, yMeas,source);
%plotResults(xForDyn, tForDyn, Xhat, P, tKalman, 0)

plotFigBaseFolder = 'plots/test/';
if(~exist(plotFigBaseFolder))
    mkdir(plotFigBaseFolder);
end

plotFigBaseName = strcat('./',plotFigBaseFolder,'predicted');

if(source == 1)
    figure(3);
    set(gca,'FontSize',12);
       set(gcf,'Renderer','OpenGL')
    print('-depsc2','-r200',strcat(plotFigBaseName,'Force_sim'),'-opengl');
    figure(4);
    set(gca,'FontSize',12);
       set(gcf,'Renderer','OpenGL')
    print('-depsc2','-r200',strcat(plotFigBaseName,'Torque_sim'),'-opengl');
    figure(5);
    set(gca,'FontSize',12);
       set(gcf,'Renderer','OpenGL')
    print('-depsc2','-r200',strcat(plotFigBaseName,'Velocitites_sim'),'-opengl');
    figure(6);
    set(gca,'FontSize',12);
       set(gcf,'Renderer','OpenGL')
    print('-depsc2','-r200',strcat(plotFigBaseName,'Orientation_sim'),'-opengl');
    
    figure(8);
    set(gca,'FontSize',12);
       set(gcf,'Renderer','OpenGL')
    print('-depsc2','-r200',strcat(plotFigBaseName,'UpdatedFRI_sim'),'-opengl');
end


if(source == 2)
    figure(1);
    set(gca,'FontSize',12);
    set(gcf,'Renderer','OpenGL');
    print('-depsc2','-r200',strcat(plotFigBaseName,'Force_real'),'-opengl');
    figure(2);
    set(gca,'FontSize',12);
    set(gcf,'Renderer','OpenGL');
    print('-depsc2','-r200',strcat(plotFigBaseName,'Torque_real'),'-opengl');
    figure(8);
    set(gca,'FontSize',12);
    set(gcf,'Renderer','OpenGL');
    print('-depsc2','-r200',strcat(plotFigBaseName,'UpdatedFRI_real'),'-opengl');
end

%Smoother
% [Xhats,Ps] = etf_smooth1(Xhat',P,y',db_dx_func,Q,b_func,[],model, dh_dx_func,R,h_func,[],model, 1, 1);
% [M,P] =        ETF_SMOOTH1(M,P,Y,          A,Q,ia,W,aparam,H,R,h,V,hparam,same_p_a,same_p_h)
% plotResults(x, Xhats', Ps, t, 4)

