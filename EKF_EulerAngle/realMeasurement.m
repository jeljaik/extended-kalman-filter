
function [yMeas,tMeas,model,R] = realMeasurement(dtKalman, model, plots, t_min, t_max, measurementType,numberOfExperiment ,whichLeg, whichSkin,dataset,experiment)
% REALMEASUREMENT_WITHSKIN loads data from the backward tipping experiments
%   including the feet skin to post-process these data.
%
%   REALMEASUREMENT_WITHSKIN(dtKalman, model, plots, t_min, t_max) When 
%   called like this, numberOfExperiments = 7, whichLeg = 'right',
%   whichSkin = 'right'.
%   REALMEASUREMENT_WITHSKIN(numberOfExperiments, whichLeg, whichSkin)

disp('processing skin and other data');

if(nargin<1)
    measurementType = 'withSkin';
    dtKalman = 0.01;
    model = struct();
    plots = 0;
    t_min = 0;
    t_max = inf;
end

if (nargin<6)
    measurementType = 'withSkin';
    numberOfExperiment = 7;
    whichLeg  = 'right';
    whichSkin = 'right';
end

fakeIMU = 'false';
fakeAccl = [ 0 ; 0 ; 9.8];
acclSign = -1;

fakeFT_f = 'false';
fakeFT_mu = 'false';
fakeMu_o = [0;0;0];
fakeF_o = [0;0;0];

%skin_data = importdata('./robotData/backwardTipping/dumperTippingSetup01/icub/skin/right_foot/data.log ');
%% Params
   %% new model parameters from robot 
 kIniTemp = model.kIni;
 if(strcmp(experiment,'leg')==1)
    [model,tMax,leg_ft,foot_ft,skin,inertial,transforms] = modelSensorParams_completeLeg(whichLeg,whichSkin,numberOfExperiment,t_max,measurementType,dataset);
 end
 if(strcmp(experiment,'foot')==1)
    [model,tMax,leg_ft,foot_ft,skin,inertial,transforms] = modelSensorParams_foot(whichLeg,whichSkin,numberOfExperiment,t_max,measurementType,dataset);
 end
 model.kIni = kIniTemp;
 model.dtKalman = dtKalman;

 t = linspace(t_min,tMax,(tMax - t_min)/dtKalman);
 tCalib = linspace(0,t_min,(t_min)/dtKalman);


%% calibration and covariance estimation data
if(strcmp(measurementType,'withoutlegFT')~=1 && strcmp(measurementType,'dualStateWithoutlegFT')~=1)
    fo_pre_calib = interp1(leg_ft.t,leg_ft.f,tCalib);
    muo_pre_calib = interp1(leg_ft.t,leg_ft.mu,tCalib);
end
fc_pre_calib = interp1(foot_ft.t,foot_ft.f,tCalib);
muc_pre_calib = interp1(foot_ft.t,foot_ft.mu,tCalib);
delta_pre_calib = interp1(skin.t,skin.data,tCalib);
a_omega_pre_calib = interp1(inertial.t,inertial.data,tCalib);

a_pre_calib = acclSign*a_omega_pre_calib(:,4:6);
omega_pre_calib = a_omega_pre_calib(:,7:9);

%Proper sensor orientation with respect to the COM frame
aPreProcMeanTest = mean(a_pre_calib',2);
[com_R_imu] = optimalIMURotation(aPreProcMeanTest)
transforms.B_R_imu =  com_R_imu;

omegaOffset = mean((omega_pre_calib'),2);

%% Total Normal Force through the skin
% The following two lines can be replaced by totalForceFromSkinData() but
% this would read again the raw values. 
delta_proc_calib = dataPostProcessing(delta_pre_calib, 'normalForces');
fc_z_calib = computeTotalForce(delta_proc_calib, 'normalForces')';

%% Forces and torques
if(strcmp(measurementType,'withoutlegFT')~=1 && strcmp(measurementType,'dualStateWithoutlegFT')~=1)
    fo_muo_calib = transforms.B_adjT_leg * [fo_pre_calib';muo_pre_calib'];
    fo_calib = fo_muo_calib(1:3,:);
    muo_calib = fo_muo_calib(4:6,:);
end
fc_muc_calib = transforms.B_adjT_ankle * [fc_pre_calib';muc_pre_calib'];
fc_calib = fc_muc_calib(1:3,:);
muc_calib = fc_muc_calib(4:6,:);


%% Force offsets corrected to calibration dataset
if(strcmp(measurementType,'withoutlegFT')~=1 && strcmp(measurementType,'dualStateWithoutlegFT')~=1)
    f_calib_delta = (0.5)*( mean(+fc_calib-fo_calib,2) - model.m* euler2dcm(model.phi0)*model.G_g);
    mu_calib_delta = (0.5)*mean(+muc_calib - muo_calib,2);
else
    f_calib_delta = (0.5)*( mean(fc_calib,2) - model.m* euler2dcm(model.phi0)*model.G_g);
    mu_calib_delta = (0.5)*(mean(muc_calib,2));
end    
%f_calib_delta = zeros(size(f_calib_delta));
%mu_calib_delta = zeros(size(mu_calib_delta));

%% IMU
a_calib = (transforms.B_R_imu*a_pre_calib');
%omegaCentered = omega_calib - repmat(omegaOffset,size(omega_pre_proc,1),1);
omega_calib = (transforms.B_R_imu*omega_pre_calib')*(pi/180);

if(strcmp(measurementType,'withoutlegFT')~=1 && strcmp(measurementType,'dualStateWithoutlegFT')~=1)
    yCalib = [a_calib;omega_calib;fo_calib;muo_calib;fc_calib;muc_calib;fc_z_calib]';
else
    yCalib = [a_calib;omega_calib;fc_calib;muc_calib;fc_z_calib]';
end
%covMat = cov(yCalib);

%R = covMat;
yCalibMean = mean(yCalib,1);
yCalibBar = yCalib - repmat(yCalibMean,size(yCalib,1),1);
R = (1/size(yCalib,2))*(yCalibBar'*yCalibBar); 




%% pre-processed interpolated data
if(strcmp(measurementType,'withoutlegFT')~=1 && strcmp(measurementType,'dualStateWithoutlegFT')~=1)
    fo_pre_proc = interp1(leg_ft.t,leg_ft.f,t);
    muo_pre_proc = interp1(leg_ft.t,leg_ft.mu,t);
end
fc_pre_proc = interp1(foot_ft.t,foot_ft.f,t);
muc_pre_proc = interp1(foot_ft.t,foot_ft.mu,t);
delta_pre_proc = interp1(skin.t,skin.data,t);
a_omega_pre_proc = interp1(inertial.t,inertial.data,t);

%% IMU and skin
a_pre_proc = acclSign*(a_omega_pre_proc(:,4:6)')';
omega_pre_proc = a_omega_pre_proc(:,7:9);

%omegaCalib = interp1(inertial.t,inertial.data(:,7:9),tCalib);
%omegaOffset = mean(omega_calib,1);

%% Total Normal Force through the skin
% The following two lines can be replaced by totalForceFromSkinData() but
% this would read again the raw values. 
delta = dataPostProcessing(delta_pre_proc, 'normalForces');
fc_z = computeTotalForce(delta, 'normalForces')';

%% Forces and torques
if(strcmp(measurementType,'withoutlegFT')~=1 && strcmp(measurementType,'dualStateWithoutlegFT')~=1)
    fo_muo = transforms.B_adjT_leg * [fo_pre_proc';muo_pre_proc'];
end
fc_muc = transforms.B_adjT_ankle * [fc_pre_proc';muc_pre_proc'];

if(strcmp(measurementType,'dualState')~=1)
   %% recomputing momment offsets to have zero initial angular acceleration if its not dualState

if(strcmp(measurementType,'withoutlegFT')~=1 && strcmp(measurementType,'dualStateWithoutlegFT')~=1)
    muo = fo_muo(4:6,:) - mean(muo_calib,2)*ones(1,length(t));
end
    
    muc = fc_muc(4:6,:) - mean(muc_calib,2)*ones(1,length(t));
else
    %% leaving offsets as they are to have an initial compliance momment
%     muo = fo_muo(4:6,:) + mu_calib_delta*ones(1,length(t));
%     muc = fc_muc(4:6,:) - mu_calib_delta*ones(1,length(t));
   
%    including offsets for the x- and z-component, since the initial
%    orientation is [0 pi/2 0]' and the initial compliance moment in these
%    directions are zero
if(strcmp(measurementType,'withoutlegFT')~=1 && strcmp(measurementType,'dualStateWithoutlegFT')~=1)
    muo = fo_muo(4:6,:);
    if(strcmp(measurementType,'dualStateWithoutlegFT')==1 || strcmp(measurementType,'dualState'))
      muo(1,:) = muo(1,:) +  mu_calib_delta(1)*ones(1,length(t));
      muo(3,:) = muo(3,:) +  mu_calib_delta(1)*ones(1,length(t));
    end
end
    muc = fc_muc(4:6,:);
    if(strcmp(measurementType,'dualStateWithoutlegFT')==1 || strcmp(measurementType,'dualState'))
    muc(1,:) = muc(1,:) +  mu_calib_delta(1)*ones(1,length(t));
    muc(3,:) = muc(3,:) +  mu_calib_delta(1)*ones(1,length(t));
    end
end

if(strcmp(measurementType,'withoutlegFT')~=1 && strcmp(measurementType,'dualStateWithoutlegFT')~=1)
fo = fo_muo(1:3,:) + f_calib_delta*ones(1,length(t));
end
fc = fc_muc(1:3,:)  - f_calib_delta*ones(1,length(t));

%% offset in skin to make it identical to FT measurements at calibrationtime
fc_zDelta = (mean(fc_z_calib - (fc_calib(3,:) - f_calib_delta(3)*ones(1,length(tCalib)))));
fc_z = fc_z - fc_zDelta;
%% IMU
a = (transforms.B_R_imu*a_pre_proc');
omegaCentered = (omega_pre_proc')' - repmat(omegaOffset',size(omega_pre_proc,1),1);
omega = (transforms.B_R_imu*omegaCentered')'.*(pi/180);


    %% plotting raw and corrected data
    if(strcmp(plots,'makePlots') == 1)
        figure(1);
            % 
           if(strcmp(measurementType,'withoutlegFT')~=1 && strcmp(measurementType,'dualStateWithoutlegFT')~=1)
            subplot(2,2,1);
            plot(t,fo_pre_proc);
            xlabel('time (sec)');
            ylabel('force (N)');
            legend('fX', 'fY', 'fZ');            
            axis tight;
            title('fo_{raw}');

            subplot(2,2,2);
            plot(t,muo_pre_proc);
            xlabel('time (sec)');
            ylabel('torque (Nm)');
            legend('muX', 'muY', 'muZ');                                    
            axis tight;
            title('muo_{raw}');
           end    
            subplot(2,2,3);
            plot(t,fc_pre_proc);
            xlabel('time (sec)');
            ylabel('force (N)');
            legend('fX', 'fY', 'fZ');                                    
            axis tight;
            title('fc_{raw}');

            subplot(2,2,4);
            plot(t,muc_pre_proc);
            xlabel('time (sec)');
            ylabel('torque (Nm)');
            legend('muX', 'muY', 'muZ');                        
            axis tight;
            title('muc_{raw}');

         figure(2);
        if(strcmp(measurementType,'withoutlegFT')~=1 && strcmp(measurementType,'dualStateWithoutlegFT')~=1)
            subplot(2,2,1);
            plot(t,fo);
            xlabel('time (sec)');
            ylabel('force (N)');
            legend('fX', 'fY', 'fZ');                        
            axis tight;
            title('fo');

            subplot(2,2,2);
            plot(t,muo);
            xlabel('time (sec)');
            ylabel('torque (Nm)');
            legend('muX', 'muY', 'muZ');            
            axis tight;
            title('muo');
         end
            subplot(2,2,3);
            plot(t,fc);
            xlabel('time (sec)');
            ylabel('force (N)');
            legend('fX', 'fY', 'fZ');            
            axis tight;
            title('fc');

            subplot(2,2,4);
            plot(t,muc); 
            xlabel('time (sec)');
            ylabel('torque (Nm)');
            legend('muX', 'muY', 'muZ');
            axis tight;
            title('muc');

        figure(3);
            %plot(t,(TFoot.ans'*del')');
            plot(t,fc_z);
            xlabel('time (sec)');
            ylabel('force (N)');
            title('Normal Force with Foot Skin');
            legend('fZ');
            axis tight;

         figure(4);
            subplot(2,1,1);
            plot(t,a_pre_proc);
            xlabel('time (sec)');
            ylabel('m/sec^2');
            legend('accX', 'accY', 'accZ');
            axis tight;
            title('Linear Acceleration a_{raw} [m/s^2]');

            subplot(2,1,2);
            plot(t,omega_pre_proc);
            xlabel('time (sec)');
            ylabel('deg/sec');
            legend('gyrX', 'gyrY', 'gyrZ');
            axis tight;
            title('Angular Velocity \omega _{raw}');

         figure(5);
            subplot(2,1,1);
            plot(t,a);
            xlabel('time (sec)');
            ylabel('m/sec^2');
            legend('accX', 'accY', 'accZ');
            axis tight;
            title('Linear Acceleration a_{com}');

            subplot(2,1,2);
            plot(t,omega);
            xlabel('time (sec)');
            ylabel('rad/sec');
            legend('gyrX', 'gyrY', 'gyrZ');
            axis tight;
            title('Angular Velocity \omega _{com}');
%             
%             figure(6);
%             subplot(2,1,1);
%             plot(t,fo-fc);
%             subplot(2,1,2);
%             plot(t,muo-muc);
    end
  %  idx = t>t_min;
  %  yMeas = [a(:,idx);omega(:,idx);fo(:,idx);muo(:,idx);fc(:,idx);muc(:,idx);fc_x(:,idx)]';
   
  if(strcmp(fakeIMU,'true') == 1)

        aPerfect = fakeAccl*ones(1,size(a,2));
        omegaPerfect= zeros(size(omega));
        a = aPerfect;
        omega = omegaPerfect;
        
        if(strcmp(plots,'makePlots') == 1)
            figure(5);
                subplot(2,1,1); hold on;
                plot(t,a,'lineWidth',2.0);
                xlabel('time (sec)');
                ylabel('m/sec^2');
                legend('accX', 'accY', 'accZ');
                axis tight;
                title('Linear Acceleration a_{com}');

                subplot(2,1,2); hold on;
                plot(t,omega,'lineWidth',2.0);
                xlabel('time (sec)');
                ylabel('rad/sec');
                legend('gyrX', 'gyrY', 'gyrZ');
                axis tight;
                title('Angular Velocity \omega _{com}');
        end
    end

    if(strcmp(fakeFT_f,'true') == 1)
       foPerfect = fakeF_o * ones(1,size(fo,2));
       
       fgPerfect = model.m*euler2dcm(model.phi0)*model.G_g * ones(1,size(fc,2));
       fcPerfect =   0.5*fgPerfect;
       foPerfect = -fcPerfect; %fakeF_o * ones(1,size(fo,2));
       
       fczPerfect = fcPerfect(3,:);
       
       
       if(strcmp(plots,'makePlots') == 1)
        
           figure(2);
            subplot(2,2,3);
            hold on; plot(t,fc,'lineWidth',2.0);
            xlabel('time (sec)');
            ylabel('force (N)');
            legend('fX', 'fY', 'fZ');            
            axis tight;
            title('fc');
            
            figure(3);hold on;
            %plot(t,(TFoot.ans'*del')');
            plot(t,fc_z,'lineWidth',2.0);
            xlabel('time (sec)');
            ylabel('force (N)');
            title('Normal Force with Foot Skin');
            legend('fZ');
            axis tight;
            
       
       end
       fo = foPerfect; 
       fc = fcPerfect;
       fc_z = fczPerfect;
    end
    if(strcmp(fakeFT_mu,'true') == 1)
       muoPerfect = fakeMu_o * ones(1,size(muo,2));
       mucPerfect = -fakeMu_o * ones(1,size(muc,2));


       muo = muoPerfect;
       muc = mucPerfect;
       
    end

  
    tMeas = t;
    % model.x0 = [zeros(3,1);omega(1,:)';fo(:,1);muo(:,1);fc(:,1);muc(:,1);[0,0.5*pi,0]']; %% corresponds to -9.81 accln on Z
   
    switch(measurementType)
        case 'withoutSkin'
            fprintf('\nSetting up withoutSkin measurement\n');            
            model.x0 = [zeros(3,1);zeros(3,1);fo(:,1);muo(:,1);fc(:,1);muc(:,1);model.phi0];     
            yMeas = [a;omega';fo;muo;fc;muc]';
        case 'withSkin'
            fprintf('\nSetting up withSkin measurement\n');            
            model.x0 = [zeros(3,1);zeros(3,1);fo(:,1);muo(:,1);fc(:,1);muc(:,1);model.phi0]; 
            yMeas = [a;omega';fo;muo;fc;muc;fc_z]';
        case 'dualState'
            fprintf('\nSetting up dualState measurement\n');            
            K0 = eye(3)*model.kIni;

            model.x0 = [zeros(3,1);zeros(3,1);fo(:,1);muo(:,1);fc(:,1);muc(:,1);model.phi0;reshape(K0,9,1)]; 
            yMeas = [a;omega';fo;muo;fc;muc;fc_z]';
        case 'withoutlegFT'  
            fprintf('\nSetting up withoutlegFT measurement\n');   
            fo = [19.3089; -3.4733; -93.4659]; %general force acting on the foot due to weight of the body
            muo = [-0.5801;-3.7921; 0.4844];   %as measured by the leg FT sensor 
            model.x0 = [zeros(3,1);zeros(3,1);fo;muo;fc(:,1);muc(:,1);model.phi0]; 
            yMeas = [a;omega';fc;muc;fc_z]';
        case 'dualStateWithoutlegFT'
            fprintf('\nSetting up dualStateWithoutlegFT measurement\n');   
            fo = [19.3089; -3.4733; -93.4659]; %general force acting on the foot due to weight of the body
            muo = [-0.5801;-3.7921; 0.4844];   %as measured by the leg FT sensor  
            K0 = eye(3)*model.kIni;
            model.x0 = [zeros(3,1);zeros(3,1);fo;muo;fc(:,1);muc(:,1);model.phi0;reshape(K0,9,1)]; 
            yMeas = [a;omega';fc;muc;fc_z]';
        otherwise
            disp('ERROR : Measurement Type Unknown');
    end
    
    

    %% testing forces and torques
    
%     fprintf('Force test \n');
%    
%     fprintf('[  fc   fo   m*B_R_G*G_g fo+m*B_g-fc ] ');
%     [fc(:,1) fo(:,1) model.m* euler2dcm(model.phi0)*model.G_g -fc(:,1)+fo(:,1)+model.m* euler2dcm(model.phi0)*model.G_g ]%          
%     fprintf('Momment test\n');
%     fprintf('[  muc   muo  ]');
%     [muc(:,1) muo(:,1)]
%     fprintf('Acceleration test \n');
%     fprintf(' [ accl , gyrs] ');
%     
%     [a(1:3,1)  omega(1,1:3)'] 
%     
%     disp('loaded measurement data');
    
    
    
    %model.m = 0.761;
    
    %model.I = [0.00253893, -4.51893e-6, -0.000903578;...
    %           -4.51893e-6,  0.00407487, 3.68679e-5;...
    %           -0.000903578, 3.68679e-5, 0.00208378];
           
            %ixx="0.00253893" ixy="-4.51893e-06" ixz="-0.000903578" iyy="0.00407487" iyz="3.68679e-05" izz="0.00208378
    
            

end

