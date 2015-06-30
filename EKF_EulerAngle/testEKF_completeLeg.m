
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


clear
close all
clc

%% adding of paths
utilities    = genpath('./utils');
symb         = genpath('./symbolicFunctions');
ellipses      = genpath('./ellipses');
dynFuncs     = genpath('./dynamicsFunctions');
plotFuncs   = genpath('./plotFunctions');
skinFuncs   = genpath('./skinFunctions');
addpath(utilities, symb, ellipses,dynFuncs,plotFuncs,skinFuncs)

%% basic experiment setup

% expID : 
% 1 - noSkin measurements and noCompliance in model
% 2 - withSkin measurements and noCompliance in model
% 3 - withSkin measurements and withCompliance in model (dualState)

%numOfExperiments = 1:3;
numOfExperiments = 3;
measurementSuffix = {'withoutSkin','withSkin','dualState'}; 
processSuffix = {'withoutCompliance','withoutCompliance','dualState'};
n       = [21,21,30];         % state dimension - (translational vel, rotational vel, w_o, w_c, RPY angle)
m       = [18,19,19 ];         % output dimension

%% Model Parameters common across experiments
%setup.dtInvDyn = 0.00001;
setup.dtForDyn = 0.0001; % EKF forward dynamics computation time step
setup.dtKalman = 0.01; % EKF computation time step (discretisation)


setup.t_min = 4.5; % time until which to calibrate
setup.t_max = 7.5; % Max time in dataset until which to filter
setup.measurementPlots = 'noPlots'; % options - 'makePlots' , 'noPlots'
setup.filterOutputPlots = 'noPlots';
setup.skipSteps = 50; % no of steps to skip for diplaying kalman execution time in loop

[kalmanQParams,kalmanRParams,kIni] = setupCovariancesForExperiments();
setup.kalmanQParams = kalmanQParams;
setup.kalmanRParams = kalmanRParams;
setup.kIni = kIni; % initial stiffness (defined but not used for all experiments)
for expID = numOfExperiments
    
    fprintf('\nProcessing measurement %s with process %s\n-------------\n\n',measurementSuffix{expID},processSuffix{expID});
   
        clearvars -except numOfExperiments measurementSuffix processSuffix expID n m setup
        close all
    
    %% Measurement model and its derivative
    f_func     = str2func(strcat('forwardDynamics_',processSuffix{expID}));
    %b_func     = @backwardDynamics;
    df_dx_func = str2func(strcat('wrapperForwardDynamicsDerivative_',processSuffix{expID}));
    dh_dx_func = str2func(strcat('wrapperOutputsDerivatives_',measurementSuffix{expID}));
    %db_dx_func = @derivativeBackwardDynamics;
    output_func = str2func(strcat('measurement_',measurementSuffix{expID}));
    h_func = @(x,model)output_func(x,model, [],[],[],[]);
    
    %% Kalman Parameters

    kalmanQParams = setup.kalmanQParams;
    
    % setting up process covariances
    kalman.a_Q  = kalmanQParams{expID}(1);%4.5;
    kalman.omega_Q  = kalmanQParams{expID}(2);%4.0;%4.75;
    kalman.f_Q  = kalmanQParams{expID}(3);%0.5;%6.5;
    kalman.mu_Q =kalmanQParams{expID}(4);%2.5;%6.5; 
    kalman.phi_Q = kalmanQParams{expID}(5);%1.5;%2.50;
    kalman.K_Q =kalmanQParams{expID}(6);

    model = setup;
    t_min = model.t_min;
    t_max = model.t_max;
    
    % realKalman.a_Q  = 4.0;%4.5;
    % realKalman.omega_Q  = 10.0;%4.0;%4.75;
    % realKalman.f_Q  = 5.0;%0.5;%6.5;
    % realKalman.mu_Q = 8.0;%2.5;%6.5; 
    % realKalman.phi_Q = 0.5;%1.5;%2.50;
    % realKalman.K_Q = 0.0075;

    kalmanRParams = setup.kalmanRParams;
    
    % setting up measurement covariances
    kalman.sigma_f = kalmanRParams{expID}(1);
    kalman.sigma_u = kalmanRParams{expID}(2);
    kalman.sigma_a = kalmanRParams{expID}(3);
    kalman.sigma_omega = kalmanRParams{expID}(4);
    kalman.sigma_skin = kalmanRParams{expID}(5);

    % realKalman.sigma_f = 1.5;       % output error variance (forces)
    % realKalman.sigma_u = 2.75;      % output error variance (torques)
    % realKalman.sigma_a = 1.25;%1.5;       % output error variance (acceleration)
    % realKalman.sigma_omega = 4.5;%1.00;
    % realKalman.sigma_skin = 25.75;


    [yMeas,tMeas,model,RData] = realMeasurement_completeLeg(model.dtKalman,model,model.measurementPlots,t_min,t_max,measurementSuffix{expID},7,'right','right');
    T = tMeas(end);
    tKalman = tMeas;
    

    switch(processSuffix{expID})
        case 'withoutCompliance'
            Q  = diag([kalman.a_Q*ones(3,1);
               kalman.omega_Q*ones(3,1);
               kalman.f_Q*ones(6,1); 
               kalman.mu_Q*ones(6,1); 
               kalman.phi_Q*ones(3,1)]);
           kalman.P = 15*diag([kalman.a_Q*ones(3,1);kalman.omega_Q*ones(3,1);kalman.f_Q*ones(3,1);kalman.mu_Q*ones(3,1);kalman.f_Q*ones(3,1);kalman.mu_Q*ones(3,1);kalman.phi_Q*ones(3,1)]);
        case 'dualState'
            Q  = diag([kalman.a_Q*ones(3,1);
               kalman.omega_Q*ones(3,1);
               kalman.f_Q*ones(6,1); 
               kalman.mu_Q*ones(6,1); 
               kalman.phi_Q*ones(3,1);
               kalman.K_Q*ones(9,1)
               ]);
           kalman.P = 15*diag([kalman.a_Q*ones(3,1);kalman.omega_Q*ones(3,1);kalman.f_Q*ones(3,1);kalman.mu_Q*ones(3,1);kalman.f_Q*ones(3,1);...
               kalman.mu_Q*ones(3,1);kalman.phi_Q*ones(3,1);kalman.K_Q*ones(9,1)]);
    end 

    %% KALMAN FILTER IMPLEMENTATION
   
    forceR = 'true';

     if(strcmp(forceR,'true')==1 || ~exist('RData'))       
         disp('Assuming an R value');
        
        switch(measurementSuffix{expID})
        case 'withoutSkin'
            R = diag([kalman.sigma_a.*ones(1,3), kalman.sigma_omega.*ones(1,3), kalman.sigma_f.*ones(1,3), kalman.sigma_u.*ones(1,3), kalman.sigma_f.*ones(1,3), kalman.sigma_u.*ones(1,3)]);
        case 'withSkin'
            R = diag([kalman.sigma_a.*ones(1,3), kalman.sigma_omega.*ones(1,3), kalman.sigma_f.*ones(1,3), kalman.sigma_u.*ones(1,3), kalman.sigma_f.*ones(1,3), kalman.sigma_u.*ones(1,3),kalman.sigma_skin.*ones(1,1)]);
        case 'dualState'
            R = diag([kalman.sigma_a.*ones(1,3), kalman.sigma_omega.*ones(1,3), kalman.sigma_f.*ones(1,3), kalman.sigma_u.*ones(1,3), kalman.sigma_f.*ones(1,3), kalman.sigma_u.*ones(1,3),kalman.sigma_skin.*ones(1,1)]);
        otherwise
            disp('ERROR : Measurement Type Unknown');
        end
     else 
         %% modify following two lines to check for without skin option
         disp('Using real data covariance matrix');
         RData(19,19) = 35.63;
         R = RData;
     end
    Ph = kalman.P;

    %% initialising EKF
    % Initializing estimate
    xh        = model.x0;
    Xhat      = zeros(n(expID),length(tKalman))';

    % Initializing update
    model.dt = model.dtKalman;
    Xupdt = zeros(length(tKalman),n(expID));
    P = zeros(size(Ph,1), size(Ph,2),length(tKalman));

    
    disp('Starting Kalman Filter prediction');
    drawnow;
    
    %% EKF execution
    for i = 1:length(tKalman)
         tic;

         % Update step
        [xh, Ph] = ekf_update1(xh , Ph, yMeas(i,:)', dh_dx_func, R,h_func, [], model);
        Xupdt(i,:) = xh;
        xAfterUpdate = xh;
        pAfterUpdate = Ph;

        % Prediction step
        [xh, Ph] =  ekf_predict1(xh, Ph, df_dx_func, Q, f_func, [], model);

        Xhat(i,:) = xh;
        P(:,:,i)  = Ph;
        if(mod(i,setup.skipSteps)==0)
            fprintf('%d Steps processing time :',setup.skipSteps);
            disp(toc());
        end       
    end

    
    plotFigBaseFolder = sprintf('./plots/humanoids2015/Exp_%d/',expID);
    dataBaseFolder = sprintf('./data/humanoids2015/Exp_%d/',expID);

    if(~exist(dataBaseFolder))
        mkdir(dataBaseFolder);
    end

    finalVersion =0;
    if(finalVersion == 1)
        close all;
    end

    save(strcat(dataBaseFolder,'filteredResult.mat'),'tKalman','yMeas','Xupdt','Xhat','P');
    if(strcmp(setup.filterOutputPlots,'makePlots') == 1)
        plotAndSaveFigs(dataBaseFolder,plotFigBaseFolder); 
    end
     
     if(expID<3)
        fprintf('\nPress any key to continue to next experiment\n');
        pause;
     end
     
end

run('compareEKF_completeLeg.m');
%Smoother
%fprintf('\n\n------------------\n\n starting EKSmoother\n');

%[Xhats,Ps] = etf_smooth1(Xhat',P,yMeas',db_dx_func,Q,b_func,[],model, dh_dx_func,R,h_func,[],model, 1, 1);
%plotResultsOutput_withSkin('',Xhats', Xhats',Ps, tKalman, yMeas,source)

%figure(8); figure(7);figure(9);
%close(5);
