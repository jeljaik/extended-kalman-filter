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


%% Model Parameters common across experiments
%setup.dtInvDyn = 0.00001;
setup.dtForDyn = 0.0001; % EKF forward dynamics computation time step
setup.dtKalman = 0.01; % EKF computation time step (discretisation)


setup.t_min = 3.0; % time until which to calibrate
setup.t_max = 7.5; % Max time in dataset until which to filter
setup.measurementPlots = 'noPlots'; % options - 'makePlots' , 'noPlots'
setup.filterOutputPlots = 'makePlots';
setup.skipSteps = 50; % no of steps to skip for diplaying kalman execution time in loop

%% toggle with dataset 
dataSet = 'old'; % options 'old' 'new'
if(strcmp(dataSet,'old') == 1)
    expNo = 7; %don't change
else (strcmp(dataSet,'new') == 1)
    expNo = 1; % options 1 - firm surface; 2 - onelayermat; 3 - twolayermat; 4 - threelayermat  
end



%% Filter 1 - EKF; 2 - JEKF; 3 - DEKF
filterID = 1:3; %1:3
filter = {'ekf','jekf','dekf'};
for i = filterID
    fprintf('\nRunning %s \n-------------\n\n',filter{i});
    
    clearvars -except filter filterID i setup dataSet expNo
    close all
    
    [kalmanQParams,kalmanRParams,kIni,cIni] = setupCovariancesForExperiments_foot(filter{i});
    setup.kalmanQParams = kalmanQParams;
    setup.kalmanRParams = kalmanRParams;
    setup.k = kIni; % initial stiffness (defined but not used for all experiments)
    setup.c = cIni; % initial damping
    setup.w = [kIni;cIni];
    if(i == 1)
        setup.filter = 'ekf';
        expID = 1:3;
        % 1 - w/o Skin w/o compliance
        % 2 - w skin w/o compliance
        % 3 -  skin w compliance
        measurementSuffix = {'withoutSkin','withSkin','withSkin'};
        processSuffix = {'withoutCompliance','withoutCompliance','withCompliance'};
        n       = [21,21,21];         % state dimension - (translational vel, rotational vel, w_o, w_c, RPY angle)
        m       = [12,13,13 ];         % output dimension
        p       = [0,0,0];
    else if(i == 2)
            setup.filter = 'jekf';
            expID = 1:2;
            % 1 - w/o Skin w compliance
            % 2 - w skin w compliance
            measurementSuffix = {'jointStateWithoutSkin','jointStateWithSkin'};
            processSuffix = {'jointState','jointState'};
            n       = [27,27];         % state dimension - (translational vel, rotational vel, w_o, w_c, RPY angle) (parameters)
            m       = [12,13];         % output dimension
            p       = [0,0];
        else if(i == 3)
                setup.filter = 'dekf';
                expID = 1:2;
                % 1 - w/o Skin w compliance
                % 2 - w skin w compliance
                measurementSuffix = {'dualStateWithoutSkin','dualStateWithSkin'};
                processSuffix = {'dualState','dualState'};
                n       = [21,21];         % state dimension - (translational vel, rotational vel, w_o, w_c, RPY angle) (parameters)
                p       = [6,6];           % parameter dimension k,c
                m       = [12,13];         % output dimension
            end
        end
    end
    
    for j = expID
        
        fprintf('\nProcessing measurement %s with process %s\n-------------\n\n',measurementSuffix{j},processSuffix{j});
        clearvars -except filter filterID i j expID measurementSuffix processSuffix n m p setup dataSet expNo
        close all
        
        %% Function handls for process mode, measurement model and their derivatives
        if(strcmp(filter{i},'ekf')==1 || strcmp(filter{i},'jekf')==1)
           
            f_func     = str2func(strcat('forwardDynamics_',processSuffix{j}));
            df_dx_func = str2func(strcat('wrapperForwardDynamicsDerivative_',processSuffix{j}));
            dh_dx_func = str2func(strcat('wrapperOutputsDerivatives_',measurementSuffix{j}));
            h_func = str2func(strcat('output_',measurementSuffix{j}));
        else if(strcmp(filter{i},'dekf')==1)
                
                f_func     = str2func(strcat('forwardDynamics_','withCompliance'));
                df_dx_func = str2func(strcat('wrapperForwardDynamicsDerivative_','withCompliance'));
                if(j == 1 )
                    dh_dx_func = str2func(strcat('wrapperOutputsDerivatives_','withoutSkin'));
                    h_func = str2func(strcat('output_','withoutSkin'));
                else if(j == 2)
                        dh_dx_func = str2func(strcat('wrapperOutputsDerivatives_','withSkin'));
                        h_func = str2func(strcat('output_','withSkin'));
                    end
                end
                w_func = str2func(strcat('paramDynamics_',processSuffix{j}));
                dw_dk_func = str2func(strcat('wrapperParamDynamicsDerivatives_',processSuffix{j}));
                dc_dw_func = str2func(strcat('wrapperParamOutputsDerivatives_',measurementSuffix{j}));
                
            end
            
        end
        %% Kalman Parameters
        kalmanQParams = setup.kalmanQParams;
        % setting up process covariances
        kalman.a_Q  = kalmanQParams(1);%4.5;
        kalman.omega_Q  = kalmanQParams(2);%4.0;%4.75;
        kalman.f_Q  = kalmanQParams(3);%0.5;%6.5;
        kalman.mu_Q =kalmanQParams(4);%2.5;%6.5;
        kalman.phi_Q = kalmanQParams(5);%1.5;%2.50;
        kalman.k_Q =kalmanQParams(6);
        kalman.c_Q =kalmanQParams(7);
        
        
        kalmanRParams = setup.kalmanRParams;
        % setting up measurement covariances
        kalman.sigma_f = kalmanRParams(1);
        kalman.sigma_u = kalmanRParams(2);
        kalman.sigma_a = kalmanRParams(3);
        kalman.sigma_omega = kalmanRParams(4);
        kalman.sigma_skin = kalmanRParams(5);
        
        
        model = setup;
        t_min = model.t_min;
        t_max = model.t_max;
        
        [yMeas,tMeas,model,RData] = realMeasurement_foot(model.dtKalman,model,model.measurementPlots,t_min,t_max,measurementSuffix{j},expNo,'right','right',dataSet);
        T = tMeas(end);
        tKalman = tMeas;
        
        %% Setting up Filter Covariances
        
        %Measurement Noise Covariance
        forceR = 'true';
        
        if(strcmp(forceR,'true')==1 || ~exist('RData'))
            disp('Assuming an R value');
            if(strcmp(measurementSuffix{j},'dualStateWithoutSkin') == 1 || strcmp(measurementSuffix{j},'jointStateWithoutSkin') == 1 || strcmp(measurementSuffix{j},'withoutSkin') == 1)
                R = diag([kalman.sigma_a.*ones(1,3), kalman.sigma_omega.*ones(1,3), kalman.sigma_f.*ones(1,3), kalman.sigma_u.*ones(1,3)]);
            else if(strcmp(measurementSuffix{j},'dualStateWithSkin') == 1 || strcmp(measurementSuffix{j},'jointStateWithSkin') == 1 || strcmp(measurementSuffix{j},'withSkin') == 1)
                    R = diag([kalman.sigma_a.*ones(1,3), kalman.sigma_omega.*ones(1,3), kalman.sigma_f.*ones(1,3), kalman.sigma_u.*ones(1,3),kalman.sigma_skin.*ones(1,1)]);
                end
            end
        else
            % modify following two lines to check for without skin option
            disp('Using real data covariance matrix');
            RData(19,19) = 35.63;
            R = RData;
        end
        
        %Process Noise and State Transition Covariance
        switch(filter{i})
            case 'ekf'
                Q  = diag([kalman.a_Q*ones(3,1);
                    kalman.omega_Q*ones(3,1);
                    kalman.f_Q*ones(3,1);
                    kalman.mu_Q*ones(3,1);
                    kalman.f_Q*ones(3,1);
                    kalman.mu_Q*ones(3,1);
                    kalman.phi_Q*ones(3,1)]);
                kalman.P = 15*diag([kalman.a_Q*ones(3,1);kalman.omega_Q*ones(3,1);kalman.f_Q*ones(3,1);kalman.mu_Q*ones(3,1);kalman.f_Q*ones(3,1);kalman.mu_Q*ones(3,1);kalman.phi_Q*ones(3,1)]);
                x0 = model.x0;
                
            case 'jekf'
                Q  = diag([kalman.a_Q*ones(3,1);
                    kalman.omega_Q*ones(3,1);
                    kalman.f_Q*ones(3,1);
                    kalman.mu_Q*ones(3,1);
                    kalman.f_Q*ones(3,1);
                    kalman.mu_Q*ones(3,1);
                    kalman.phi_Q*ones(3,1);
                    kalman.k_Q*ones(3,1);
                    kalman.c_Q*ones(3,1)]);
                
                kalman.P = 15*diag([kalman.a_Q*ones(3,1);kalman.omega_Q*ones(3,1);kalman.f_Q*ones(3,1);kalman.mu_Q*ones(3,1);kalman.f_Q*ones(3,1);...
                    kalman.mu_Q*ones(3,1);kalman.phi_Q*ones(3,1);kalman.k_Q*ones(3,1);kalman.c_Q*ones(3,1)]);
                x0 = model.x0;
            case 'dekf'
                Q  = diag([kalman.a_Q*ones(3,1);
                    kalman.omega_Q*ones(3,1);
                    kalman.f_Q*ones(3,1);
                    kalman.mu_Q*ones(3,1);
                    kalman.f_Q*ones(3,1);
                    kalman.mu_Q*ones(3,1);
                    kalman.phi_Q*ones(3,1)]);
                kalman.P = 15*diag([kalman.a_Q*ones(3,1);kalman.omega_Q*ones(3,1);kalman.f_Q*ones(3,1);kalman.mu_Q*ones(3,1);kalman.f_Q*ones(3,1);kalman.mu_Q*ones(3,1);kalman.phi_Q*ones(3,1)]);
                
                Q_param = diag([kalman.k_Q*ones(3,1);...
                    kalman.c_Q*ones(3,1)]);
                
                P_param = diag([kalman.k_Q*ones(3,1);...
                    kalman.c_Q*ones(3,1)]);
                
                R_param = R;
                x0 = model.x0;
                w0 = model.w0;
        end
        
        
        
        %% KALMAN FILTER IMPLEMENTATION
        %% initialising EKF
        % Initializing estimate and update
        model.dt = model.dtKalman;
        Ph = kalman.P;
        xh        = x0;
        
        if(strcmp(filter{i},'dekf') == 1)
            wh = w0;
            Pw = P_param;
            Xhat      = zeros(n(j)+p(j),length(tKalman))';
            % Initializing update
            Xupdt = zeros(length(tKalman),n(j)+p(j));
            P = zeros(n(j), n(j),length(tKalman));
            P_p = zeros(p(j), p(j),length(tKalman));
        else
            Xhat      = zeros(n(j),length(tKalman))';
            Xupdt = zeros(length(tKalman),n(j));
            P = zeros(size(Ph,1), size(Ph,2),length(tKalman));
            
        end
        
        
        disp('Starting Kalman Filter prediction');
        drawnow;
        
        %% EKF execution
        for k = 1:length(tKalman)
            tic;
            % Update step
            [xh, Ph] = ekf_update1(xh , Ph, yMeas(k,:)', dh_dx_func, R,h_func, [], model);
            
            if(strcmp(filter{i},'dekf') ~= 1)
                Xupdt(k,:) = xh;
            end
            
            Ph = (Ph + Ph')/2;
            xAfterUpdate = xh;
            pAfterUpdate = Ph;
            
            if(strcmp(filter{i},'dekf') == 1)
                %param update
                [wh, Pw] = ekf_updateparam1(xh,wh,Pw,yMeas(k,:)',dh_dx_func,dc_dw_func,R_param,h_func, [], model);
                model.w = wh;
                Xupdt(k,:) = [xh;wh];
                Pw = (Pw + Pw')/2';
                
                %param predict
                [wh, Pw] = ekf_predictparam1(xh,wh, Pw,dw_dk_func,Q_param,w_func,[], model);
                model.w = wh;
                
            end
            
            
            % Prediction step
            [xh, Ph] =  ekf_predict1(xh, Ph, df_dx_func, Q, f_func, [], model);
            
            if(strcmp(filter{i},'dekf') ~= 1)
                Xhat(k,:) = xh;
                P(:,:,k)  = Ph;
            else if(strcmp(filter{i},'dekf') == 1)
                    
                    Xhat(k,:) = [xh;wh];
                    P(:,:,k)  = Ph;
                    P_p(:,:,k) = Pw;
                end
            end
            
            if(mod(k,model.skipSteps)==0)
%                         fprintf('\n %d Steps processing time : ',model.skipSteps);
%                         disp(toc());
                fprintf('\n Time  : %d \n',t_min + (k*model.dtKalman));
            end
            
        end
        
        plotFigBaseFolder = sprintf('./plots/%s/%s%s/',filter{i},processSuffix{j},measurementSuffix{j});
        dataBaseFolder = sprintf('./data/%s/%s%s/',filter{i},processSuffix{j},measurementSuffix{j});
        
        if(~exist(dataBaseFolder))
            mkdir(dataBaseFolder);
        end
        
        finalVersion =0;
        if(finalVersion == 1)
            close all;
        end
        
        save(strcat(dataBaseFolder,'filteredResult.mat'),'tKalman','yMeas','Xupdt','Xhat','P');
        if(strcmp(model.filterOutputPlots,'makePlots') == 1)
            plotAndSaveFigs(dataBaseFolder,plotFigBaseFolder,model.filter);
        end
        
        if(j < 4)
            fprintf('\nPress any key to continue to next experiment\n');
            pause;
        end
        
        
    end
    
end

