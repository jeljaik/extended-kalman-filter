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

%% Choice of dataset
whichDataset = 'old' ; %options - 'old', 'new'
if(strcmp(whichDataset,'old')==1)
    trialNum = 7; 
else if (strcmp(whichDataset,'new')==1)
%     trialNum = 1:4; %options 1: firm surface 2: 1-layer mat 3: 2-layer mat 4: 3-layer mat   
    trialNum = 1:4;
    end
end
%% Choice of estimation - wrenches acting on Leg COM or Foot COM
experiment = 'leg'; %options - 'leg', 'foot'

%% basic experiment setup
if(strcmp(experiment,'foot')==1)
    % expID :
    % 1 - nolegFT measurements and noCompliance in model 
    % 2 - nolegFT measurements and Compliance in model
    % numOfExperiments = 1:2;
    numOfExperiments = 2;
    measurementSuffix = {'withoutlegFT','dualStateWithoutlegFT'};
    processSuffix = {'withoutCompliance','dualState'};
    n = [21,30];
    m = [13,13]; %output dimension [acc(3),gyr(3,)ankle ft(6),skin(1)]
    disp('Considering Wrenches acting on the Foot alone');
end

if(strcmp(experiment,'leg')==1)
    % expID : 
    % 1 - noSkin measurements and noCompliance in model
    % 2 - withSkin measurements and noCompliance in model
    % 3 - withSkin measurements and withCompliance in model (dualState)
    % 4 - nolegFT measurements and noCompliance in model 
    % 5 - nolegFT measurements and Compliance in model

    %numOfExperiments = 1:5;
    numOfExperiments = 1:3;
    measurementSuffix = {'withoutSkin','withSkin','dualState','withoutlegFT','dualStateWithoutlegFT'}; 
    processSuffix = {'withoutCompliance','withoutCompliance','dualState','withoutCompliance','dualState'};
    n       = [21,21,30,21,30];         % state dimension - (translational vel, rotational vel, w_o, w_c, RPY angle)
    m       = [18,19,19,13,13];         % output dimension
    disp('Considering Wrenches acting on the whole Leg');
end


%% Model Parameters common across experiments
%setup.dtInvDyn = 0.00001;
setup.dtForDyn = 0.0001; % EKF forward dynamics computation time step
setup.dtKalman = 0.01; % EKF computation time step (discretisation)

setup.t_min = 3.0; % time until which to calibrate
setup.t_max = 7.5; % Max time in dataset until which to filter
setup.measurementPlots = 'noPlots'; % options - 'makePlots' , 'noPlots'
setup.filterOutputPlots = 'noPlots'; % options - 'makePlots' , 'noPlots'
setup.comparePlots = 'makePlots';
setup.skipSteps = 50; % no of steps to skip for diplaying kalman execution time in loop

[kalmanQParams,kalmanRParams,kIni] = setupCovariancesForExperiments_comprehensive(experiment); 
setup.kalmanQParams = kalmanQParams;
setup.kalmanRParams = kalmanRParams;
setup.kIni = kIni; % initial stiffness (defined but not used for all experiments)



for dataID = trialNum
for expID = numOfExperiments
    
    if(strcmp(whichDataset,'new')==1)
        disp('new dataset Trial 1 - firm surface, Trial 2 - 1-layer soft mat, Trial 3 - 2-layer mat, Trial 4 - 3-layer mat');
    end
    if(strcmp(whichDataset,'old')==1)
        disp('old dataset Trial 7');
    end
    fprintf('\n Reading %s dataset, Trial number : %d \n',whichDataset,dataID);
    fprintf('\nProcessing measurement %s with process %s\n-------------\n\n',measurementSuffix{expID},processSuffix{expID});
   
        clearvars -except numOfExperiments measurementSuffix processSuffix expID n m setup whichDataset trialNum experiment dataID
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
    kalman.K_Q = kalmanQParams{expID}(6);

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

    
        [yMeas,tMeas,model,RData] = realMeasurement_comprehensive(model.dtKalman,model,model.measurementPlots,t_min,t_max,measurementSuffix{expID},dataID,'right','right',whichDataset,experiment);
 
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
        case 'withoutlegFT'
            R = diag([kalman.sigma_a.*ones(1,3), kalman.sigma_omega.*ones(1,3), kalman.sigma_f.*ones(1,3), kalman.sigma_u.*ones(1,3),kalman.sigma_skin.*ones(1,1)]);
        case 'dualStateWithoutlegFT'
            R = diag([kalman.sigma_a.*ones(1,3), kalman.sigma_omega.*ones(1,3), kalman.sigma_f.*ones(1,3), kalman.sigma_u.*ones(1,3),kalman.sigma_skin.*ones(1,1)]);   
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


    if(strcmp(experiment,'leg')==1)
        if(strcmp(whichDataset,'old')==1)
%             plotFigBaseFolder = sprintf('./plots/humanoids2015/leg/old/trial_%d/Exp_%d/',dataID,expID);
            dataBaseFolder = sprintf('./data/humanoids2015/leg/old/trial_%d/Exp_%d/',dataID,expID);
            plotFigBaseFolder = sprintf('./plots/humanoids2015/');
        end   
        if(strcmp(whichDataset,'new')==1)
%             plotFigBaseFolder = sprintf('./plots/humanoids2015/leg/new/trial_%d/Exp_%d/',dataID,expID);
            dataBaseFolder = sprintf('./data/humanoids2015/leg/new/trial_%d/Exp_%d/',dataID,expID);
            plotFigBaseFolder = sprintf('./plots/humanoids2015/');
        end   
    end
    
    if(strcmp(experiment,'foot')==1)
        if(strcmp(whichDataset,'old')==1)
%             plotFigBaseFolder = sprintf('./plots/humanoids2015/foot/old/trial_%d/Exp_%d/',dataID,expID);
            dataBaseFolder = sprintf('./data/humanoids2015/foot/old/trial_%d/Exp_%d/',dataID,expID);
            plotFigBaseFolder = sprintf('./plots/humanoids2015/');
        end
        if(strcmp(whichDataset,'new')==1)
            plotFigBaseFolder = sprintf('./plots/humanoids2015/foot/new/trial_%d/Exp_%d/',dataID,expID);
%             dataBaseFolder = sprintf('./data/humanoids2015/foot/new/trial_%d/Exp_%d/',dataID,expID);
            plotFigBaseFolder = sprintf('./plots/humanoids2015/');
        end
    end
    
    
    if(~exist(dataBaseFolder))
        mkdir(dataBaseFolder);
    end

    finalVersion =0;
    if(finalVersion == 1)
        close all;
    end

    
    save(strcat(dataBaseFolder,'filteredResult.mat'),'tKalman','yMeas','Xupdt','Xhat','P');
    save(strcat(dataBaseFolder,'setup.mat'),'-struct','setup')
    
    if(strcmp(setup.filterOutputPlots,'makePlots') == 1)
        plotAndSaveFigs(dataBaseFolder,plotFigBaseFolder); 
    end
     
           
    
    if(expID<3)
        fprintf('\nPress any key to continue to next experiment\n\n');
        pause;
     end
     
end
end
plotAndSaveHumanoidsFigs(plotFigBaseFolder); 
%This function is used to compare various plots for different experiments.
% Note the sequence of input vectors which reflects what you want to
% compare. Also beware that all the input vectors except variable 'compare' are of the same length.
% experiment - 'leg', 'foot'
% whichDataset - 'old', 'new'
% trialNum - 1,2,3,4 (new), 7(old)
% expID - 1,2 (foot), 1,2,3,4,5(leg)
% processType - 'withoutCompliance','dualState'
% measurementType - 'withoutSkin','withSkin',,'dualState','withoutlegFT','dualStateWithoutlegFT'
% plotDivide - 'true','false' : true gives steady state and dynamic state
% behavior individually
% legends - as a vector of same length
% compare - 'orientation','orientationvariance', 'stiffness' , 'FRI' , 'all'


% comparePlots(experiment,whichDataset,trialNum,expID,processType,measurementType,plotDivide,legends)
% experiment = cellstr(['foot';'foot';'foot';'foot']);
% whichDataset = cellstr(['new';'new';'new';'new']);
% trialNum = [1;2;3;4];
% expID = [2;2;2;2];
% processType = cellstr(['dualState';'dualState';'dualState';'dualState']);
% measurementType = cellstr(['dualStateWithoutlegFT';'dualStateWithoutlegFT';'dualStateWithoutlegFT';'dualStateWithoutlegFT']);
% plotDivide = cellstr(['false';'false';'false';'false']);
% legends = cellstr(['Firm   ';'1-layer';'2-layer';'3-layer']);
% compare = 'orientation'; 
% 
% 
% if(strcmp(setup.comparePlots,'makePlots') == 1)
%     comparePlots(experiment,whichDataset,trialNum,expID,processType,measurementType,plotDivide,legends,compare);
% end

%Smoother
%fprintf('\n\n------------------\n\n starting EKSmoother\n');

%[Xhats,Ps] = etf_smooth1(Xhat',P,yMeas',db_dx_func,Q,b_func,[],model, dh_dx_func,R,h_func,[],model, 1, 1);
%plotResultsOutput_withSkin('',Xhats', Xhats',Ps, tKalman, yMeas,source)

%figure(8); figure(7);figure(9);
%close(5);
