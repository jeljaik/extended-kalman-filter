
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
f_func     = @forwardDynamics; %process model of the state variable
b_func     = @backwardDynamics;
k_func     = @paramDynamics; %parameter dynamics, basically dK = 0 

df_dx_func = @derivativeForwardDynamics;
dk_dw_func = @derivativeParamDynamics;
dh_dx_func = @outputDerivatives;
dc_dw_func = @paramOutputs; %parameter output jacoobian C = H* do(f)/do(w)
db_dx_func = @derivativeBackwardDynamics;
h_func = @(x,model)rigidBodyOutput(x,model, [],[],[],[]);

source = 2; % 1 : sim data, 2 : real-data

%T       = 2;        % estimation time span
n       = 21;         % state dimension - (translational vel, rotational vel, w_o, w_c, RPY angle)
m       = 19;         % output dimension
p       = 9;           %parameter dimension

%% Kalman Parameters
% RealSensor parameters

%dt      = 0.01;      % sampling time
realKalman.T       = 2;       % time span
realKalman.sigma_f = 1.5;       % output error variance (forces)
realKalman.sigma_u = 2.75;      % output error variance (torques)
realKalman.sigma_a = 1.25;%1.5;       % output error variance (acceleration)
realKalman.sigma_omega = 4.5;%1.00;
realKalman.sigma_skin = 25.75;

realKalman.a_Q  = 4.0;%4.5;
realKalman.omega_Q  = 10.0;%4.0;%4.75;
realKalman.f_Q  = 5.0;%0.5;%6.5;
realKalman.mu_Q = 8.0;%2.5;%6.5; 
realKalman.phi_Q = 0.5;%1.5;%2.50;

realKalman.K_Q = 0.45; %stiffness noice covariance 


% State covariance matrix
realKalman.P = 150*diag([realKalman.a_Q*ones(3,1);realKalman.omega_Q*ones(3,1);realKalman.f_Q*ones(3,1);realKalman.mu_Q*ones(3,1);realKalman.f_Q*ones(3,1);realKalman.mu_Q*ones(3,1);realKalman.phi_Q*ones(3,1)]);
%Parameter covariance matrix
realKalman.P_param =  diag(ones(9,1));


%% Model Parameters
model.dtInvDyn = 0.00001;
model.dtForDyn = 0.0001;
model.dtKalman = 0.01;%0.05;%0.0025;%0.01;

t_min = 6.0;%4.5;%61;%42;
t_max = 7.5;
%t_max = 8.0;%8.0;%8.25;%64;%43;

[yMeas,tMeas,model,RData] = realMeasurement_completeLegWithSkin(model.dtKalman,model,0,t_min,t_max);
T = tMeas(end);
tKalman = tMeas;
kalman = realKalman;


%% KALMAN FILTER IMPLEMENTATION
%Process noise covariance for the state filter
Q  = diag([kalman.a_Q*ones(3,1);
           kalman.omega_Q*ones(3,1);
           kalman.f_Q*ones(6,1); 
           kalman.mu_Q*ones(6,1); 
           kalman.phi_Q*ones(3,1)]);

%Parameter noise covariance for the parameter filter filter
Q_param = diag(kalman.K_Q*ones(9,1));



%Measurement noice covariances
    R =diag([kalman.sigma_a.*ones(1,3), kalman.sigma_omega.*ones(1,3), kalman.sigma_f.*ones(1,3), kalman.sigma_u.*ones(1,3), kalman.sigma_f.*ones(1,3), kalman.sigma_u.*ones(1,3),kalman.sigma_skin.*ones(1,1)]);
    R_param = diag(ones(19,1));

%Iniitializing state and parameter covariances
Ph = kalman.P;
Pl = kalman.P_param;
% Initializing estimate
xh        = model.x0;% + 0.1*randn(size(model.x0));
wh        = model.w0;
Xhat      = zeros(n,length(tKalman))';
What      = zeros(p,length(tKalman))';

% Initializing update
model.dt = model.dtKalman;
Xupdt = zeros(length(tKalman),n);
Wupdt = zeros(length(tKalman),p);
P = zeros(size(Ph,1), size(Ph,2),length(tKalman));
Pw = zeros(size(Pl,1), size(Pl,2),length(tKalman));

disp('Starting Kalman Filter prediction');
drawnow;

%fprintf('initial : \n');
%disp(model.x0(1:3)'); disp(kalman.P(1:6,1:3));

for i = 1:length(tKalman)
    if(mod(i,25)==0)
        fprintf('Timenow : ');disp(tKalman(i)); drawnow();
    end
    tic;
    % Update step
    % [xe, Pe, e, Lambda] = updateStepKF(xn', y(i-1,:)', C, Pn, R, model);
    %[xh, Ph] = ekf_update1(xh , Ph, y(i,:)', dh_dx_func, R, h_func, [], model);
    [xh, Ph] = ekf_update1(xh , Ph, yMeas(i,:)', dh_dx_func, R,...
        h_func, [], model);
    
    
    %PARAMETER update : uses a function ekf_updateparam1, derivd from the function ekf_update1
    % can be significantly improved
    % function [M,P,K,MU,S,LH] = ekf_updateparam1(X,M,P,y,H,C,R,h,V,param)
    % X : state estimate from the state filter
    % M : parameter estimate
    % H : State Measurement jacobian
    % C : Parameter Measurement Jacobian
    [wh, Pl] = ekf_updateparam1(xh,wh,Pl,yMeas(i,:)',dh_dx_func,dc_dw_func,R_param,h_func, [], model);
    
    
  %  fprintf('after update : \n');
  %  disp(xh(1:3)'); disp(Ph(1:3,1:3));
    Xupdt(i,:) = xh;
    xAfterUpdate = xh;
    Wupdt(i,:) = wh;
    wAfterUpdate = wh;
    
    %Symmetrizing the covariances
    Ph = (Ph + Ph')/2;
    Pl = (Pl + Pl')/2;
    pAfterUpdate = Ph;
    plAfterUpdate = Pl;
    
    % Prediction step update
    %PARAMETER prediction
    %uses function ekf_predictparam1 accepts state, parameter inputs
    [wh, Pl] = ekf_predictparam1(xh,wh, Pl,dk_dw_func,Q_param,k_func,[], model);
    model.K = reshape(wh,3,3);
    
    % [xn, Pn] = predictStepKF(Xhat(i-1,:)', P(:,:,i-1),          A, Q, model);
    [xh, Ph] =  ekf_predict1(xh, Ph, df_dx_func, Q, f_func, [], model);
    
%     fprintf('after predict : \n');
%     disp(xh(1:3)'); disp(Ph(1:3,1:3));
%     
    Xhat(i,:) = xh;
    P(:,:,i)  = Ph;
       % pause;
    
end

%dataBaseFolder = './data/irosMain/';
%plotFigBaseFolder = 'plots/acclTests/';
%dataBaseFolder = './data/acclTests/';
plotFigBaseFolder = './plots/eksmoother/';
dataBaseFolder = './data/eksmoother/';

if(~exist(dataBaseFolder))
    mkdir(dataBaseFolder);
end

finalVersion =0;
if(finalVersion == 1)
    close all;
end

%plotResultsOutput_withSkin([],Xupdt, Xhat,Ph, tKalman, yMeas,source)
% 
 save(strcat(dataBaseFolder,'filter_result_data.mat'),'tKalman','yMeas','Xupdt','Xhat','P');
 plotAndSaveFigs(dataBaseFolder,plotFigBaseFolder);
% % %         
% % if(source == 1)
% %     figure(3);
% %     set(gca,'FontSize',12);
% %        set(gcf,'Renderer','OpenGL')
% %     print('-depsc2','-r200',strcat(plotFigBaseName,'Force_sim'),'-opengl');
% %     figure(4);
% %     set(gca,'FontSize',12);
% %        set(gcf,'Renderer','OpenGL')
% %     print('-depsc2','-r200',strcat(plotFigBaseName,'Torque_sim'),'-opengl');
% %     figure(5);
% %     set(gca,'FontSize',12);
% %        set(gcf,'Renderer','OpenGL')
% %     print('-depsc2','-r200',strcat(plotFigBaseName,'Velocitites_sim'),'-opengl');
% %     figure(6);
% %     set(gca,'FontSize',12);
% %        set(gcf,'Renderer','OpenGL')
% %     print('-depsc2','-r200',strcat(plotFigBaseName,'Orientation_sim'),'-opengl');
% %     
% %     figure(8);
% %     set(gca,'FontSize',12);
% %        set(gcf,'Renderer','OpenGL')
% %     print('-depsc2','-r200',strcat(plotFigBaseName,'UpdatedFRI_sim'),'-opengl');
% % end



%Smoother
%[Xhats,Ps] = etf_smooth1(Xhat',P,yMeas',db_dx_func,Q,b_func,[],model, dh_dx_func,R,h_func,[],model, 1, 1);
% plotResults(x, Xhats', Ps, t, 4)
%plotResultsOutput_withSkin('',Xhats', Xhats',Ps, tKalman, yMeas,source)

%figure(8); figure(7);figure(9);
%close(5);
