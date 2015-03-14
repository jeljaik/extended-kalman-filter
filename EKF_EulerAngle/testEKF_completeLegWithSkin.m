
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
symb         = genpath('./symbolicFunctions');
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

%T       = 2;        % estimation time span
n       = 21;         % state dimension - (translational vel, rotational vel, w_o, w_c, RPY angle)
m       = 19;         % output dimension

%% Kalman Parameters
% RealSensor parameters

%dt      = 0.01;      % sampling time
realKalman.T       = 2;       % time span
realKalman.sigma_f = 1.5;       % output error variance (forces)
realKalman.sigma_u = 2.75;      % output error variance (torques)
realKalman.sigma_a = 1.25;%1.5;       % output error variance (acceleration)
realKalman.sigma_omega = 4.5;%1.00;
realKalman.sigma_skin = 25.75;

realKalman.a_Q  = 0.5;%4.5;
realKalman.omega_Q  = 6.0;%4.75;
realKalman.f_Q  = 0.5;%6.5;
realKalman.mu_Q = 2.5;%6.5; 
realKalman.phi_Q = 2.50;

%realKalman.P = 0.001*diag([10*ones(6,1); 400*ones(6,1); 10*ones(6,1);20*ones(3,1)]);
%realKalman.P =5.0* diag([300*ones(3,1);50*ones(3,1); 50*ones(3,1);10*ones(3,1); 50*ones(3,1);10*ones(3,1); 50*ones(3,1)]);
realKalman.P =25.0* diag([10.0*ones(3,1);85.0*ones(3,1); 10.0*ones(3,1);10.0*ones(3,1); 25.0*ones(3,1);25.0*ones(3,1); 25*ones(3,1)]);
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
simKalman.P = diag(50*[100*ones(6,1); ones(6,1); ones(6,1); ones(3,1)]);

%P = diag([simKalman.sigma_a.*ones(1,3),simKalman.sigma_f.*ones(1,3), simKalman.sigma_f.*ones(1,3), simKalman.sigma_u.*ones(1,3), simKalman.sigma_u.*ones(1,3)]);

%% Model Parameters
model.I   = diag([0.05 0.02 0.03]);
model.m   = 0.761;%7;
model.dtInvDyn = 0.00005;
model.dtForDyn = 0.0001;
model.dtKalman = 0.01;%0.05;%0.0025;%0.01;
model.g   = 9.81;

model.bck = false;

t_min = 4.5;%61;%42;
t_max = 8.0;%8.0;%8.25;%64;%43;



%% chose source of data (simulation or real-robot)
if(source ==1) 
    kalman = simKalman;
    R =diag([kalman.sigma_a.*ones(1,3),kalman.sigma_omega.*ones(1,3),kalman.sigma_f.*ones(1,3), kalman.sigma_u.*ones(1,3),kalman.sigma_f.*ones(1,3), kalman.sigma_u.*ones(1,3), kalman.sigma_skin*ones(1,1)]);
    tKalman = 0:model.dtKalman:T;
   [yMeas,model] = simulatedMeasurement(tKalman,R,model,'forceSim',1); % set the last parameter to empty to use saved simulation data if exists
   % [yMeas,model] = simulatedMeasurement(tKalman,R,model,[],1); % set the last parameter to empty to use saved simulation data if exists
    
    
else
    [yMeas,tMeas,model,RData] = realMeasurement_completeLegWithSkin(model.dtKalman,model,0,t_min,t_max);
    T = tMeas(end);
   tKalman = tMeas;
   % numSamples = length(tKalman) - 4000;
   %numJump = 1;%10;
    
  %  yMeas = yMeas(numSamples:end,:);tKalman = tKalman(numSamples:end);
  % yMeas = yMeas(1:numJump:end,:);
  % tKalman = tKalman(1:numJump:end);
  kalman = realKalman;
end
% remove next line later
%kalman.P = diag([kalman.sigma_a.*ones(1,3),kalman.sigma_f.*ones(1,3), kalman.sigma_f.*ones(1,3), kalman.sigma_u.*ones(1,3), kalman.sigma_u.*ones(1,3)]);


%% KALMAN FILTER IMPLEMENTATION
Q  = diag([kalman.a_Q*ones(3,1);
           kalman.omega_Q*ones(3,1);
           kalman.f_Q*ones(6,1); 
           kalman.mu_Q*ones(6,1); 
           kalman.phi_Q*ones(3,1)]);
%Q = Q - 25*rand(size(Q));


%if(~exist('RData'))       
    R =1.5*diag([kalman.sigma_a.*ones(1,3), kalman.sigma_omega.*ones(1,3), kalman.sigma_f.*ones(1,3), kalman.sigma_u.*ones(1,3), kalman.sigma_f.*ones(1,3), kalman.sigma_u.*ones(1,3),kalman.sigma_skin.*ones(1,1)]);
%else 
%    disp('Using real data covariance matrix');
%    RData(19,19) = 35.63;
%    R = RData;
%end
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

fprintf('initial : \n');
disp(model.x0(1:3)'); disp(kalman.P(1:6,1:3));

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

  %  fprintf('after update : \n');
  %  disp(xh(1:3)'); disp(Ph(1:3,1:3));
    Xupdt(i,:) = xh;
    xAfterUpdate = xh;
    pAfterUpdate = Ph;
    
    % Prediction step update
    % [xn, Pn] = predictStepKF(Xhat(i-1,:)', P(:,:,i-1),          A, Q, model);
    [xh, Ph] =  ekf_predict1(xh, Ph, df_dx_func, Q, f_func, [], model);
    
%     fprintf('after predict : \n');
%     disp(xh(1:3)'); disp(Ph(1:3,1:3));
%     
    Xhat(i,:) = xh;
    P(:,:,i)  = Ph;
    if(mod(i,10)==0)
        fprintf('Step processing time :');
        disp(toc());
        fprintf('pred fo muo\n');
        disp(Xupdt(i,7:9));
        fprintf('expect fo muo\n');
        disp(Xhat(i,7:9));
        
    end
   % pause;
    
end

%dataBaseFolder = './data/irosMain/';
plotFigBaseFolder = 'plots/acclTests/';
dataBaseFolder = './data/acclTests/';
if(~exist(dataBaseFolder))
    mkdir(dataBaseFolder);
end

finalVersion =1;
if(finalVersion == 1)
    close all;
end

save(strcat(dataBaseFolder,'filter_result_data.mat'),'tKalman','yMeas','Xupdt','Xhat','P');
plotAndSaveFigs(dataBaseFolder,plotFigBaseFolder);
        
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



%Smoother
% [Xhats,Ps] = etf_smooth1(Xhat',P,y',db_dx_func,Q,b_func,[],model, dh_dx_func,R,h_func,[],model, 1, 1);
% [M,P] =        ETF_SMOOTH1(M,P,Y,          A,Q,ia,W,aparam,H,R,h,V,hparam,same_p_a,same_p_h)
% plotResults(x, Xhats', Ps, t, 4)

figure(8); figure(7);figure(9);
