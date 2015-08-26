
function [yMeas,tMeas,model,R,phiTrue] = realMeasurement_completeLeg(dtKalman, model, plots, t_min, t_max, measurementType,numberOfExperiment ,whichLeg, whichSkin)
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
    t_min = 0.0;
    t_max = inf;
end

if (nargin<6)
    measurementType = 'withSkin';
    numberOfExperiment = 7;
    whichLeg  = 'right';
    whichSkin = 'right';
end

fakeFlags.fakeIMU = 'false';
fakeFlags.fakeFT_f = 'false';
fakeFlags.fakeFT_mu = 'false';

fakeValues.fakeAccl =[ 0 ; 0 ; 9.8];
fakeValues.fakeF_o = [0;0;0];
fakeValues.fakeMu_o = [0;0;0];

%skin_data = importdata('./robotData/backwardTipping/dumperTippingSetup01/icub/skin/right_foot/data.log ');
%% Params
%% new model parameters from robot 
kIniTemp = model.kIni;
[model,tMax,leg_ft,foot_ft,skin,inertial,transforms] = modelSensorParams_completeLeg(whichLeg,whichSkin,model,t_max);
model.kIni = kIniTemp;
model.dtKalman = dtKalman;
%% Rototranslation definitions
model.phi0 = [0;0.5*pi;0];% [0;0.5*pi;0];
t = linspace(t_min,tMax,(tMax - t_min)/dtKalman);

tCalib = linspace(0,t_min,(t_min)/dtKalman);


%% calibration and covariance estimation data
fo_calib = interp1(leg_ft.t,leg_ft.f,tCalib);
muo_calib = interp1(leg_ft.t,leg_ft.mu,tCalib);
fc_calib = interp1(foot_ft.t,foot_ft.f,tCalib);
muc_calib = interp1(foot_ft.t,foot_ft.mu,tCalib);
delta_calib = interp1(skin.t,skin.data,tCalib);
a_omega_calib = interp1(inertial.t,inertial.data,tCalib);

a_calib = a_omega_calib(:,4:6);
omega_calib = a_omega_calib(:,7:9);

omegaOffset = mean((omega_calib'),2); % since in calibration phase there is expected to be zero omega;

%% Total Normal Force through the skin
% The following two lines can be replaced by totalForceFromSkinData() but
% this would read again the raw values. 
delta_proc_calib = dataPostProcessing(delta_calib, 'normalForces');
fc_z_calib = computeTotalForce(delta_proc_calib, 'normalForces')';

%% Forces and torques
fo_muo_calib = transforms.B_adjT_leg * [fo_calib';muo_calib'];
fo_calib = fo_muo_calib(1:3,:);
muo_calib = fo_muo_calib(4:6,:);

fc_muc_calib = transforms.B_adjT_ankle * [fc_calib';muc_calib'];
fc_calib = fc_muc_calib(1:3,:);
muc_calib = fc_muc_calib(4:6,:);


%% Force offsets corrected to calibration dataset
f_calib_delta = -(0.5)*( mean(+fo_calib-fc_calib,2) + model.m* euler2dcm(model.phi0)*model.G_g);
mu_calib_delta = -(0.5)*mean(+muo_calib - muc_calib,2);

%% IMU

% recomputing the com_R_imu
transforms.B_R_imu = computeOptimalIMURotation(model.dataSource,mean(a_calib)','MakePlots','on');

a_calib = (transforms.B_R_imu*a_calib');
omega_calib = (transforms.B_R_imu*omega_calib')*(pi/180);

yCalib = [a_calib;omega_calib;fo_calib;muo_calib;fc_calib;muc_calib;fc_z_calib]';

%% obtaining covariance of calibration dataset
%covMat = cov(yCalib);
yCalibMean = mean(yCalib,1);
yCalibBar = yCalib - repmat(yCalibMean,size(yCalib,1),1);
R = (1/size(yCalib,2))*(yCalibBar'*yCalibBar); 


%% raw interpolated data
fo_raw = interp1(leg_ft.t,leg_ft.f,t);
muo_raw = interp1(leg_ft.t,leg_ft.mu,t);
fc_raw = interp1(foot_ft.t,foot_ft.f,t);
muc_raw = interp1(foot_ft.t,foot_ft.mu,t);
delta_raw = interp1(skin.t,skin.data,t);
a_omega_raw = interp1(inertial.t,inertial.data,t);

%% Processing the raw data
%% IMU and skin
a_raw = a_omega_raw(:,4:6);
omega_raw = a_omega_raw(:,7:9);

rawData.fo = fo_raw;
rawData.muo = muo_raw;
rawData.fc = fc_raw;
rawData.muc = muc_raw;
rawData.a = a_raw;
rawData.omega = omega_raw;

%% Total Normal Force through the skin
% The following two lines can be replaced by totalForceFromSkinData() but
% this would read again the raw values. 
delta = dataPostProcessing(delta_raw, 'normalForces');
fc_z = computeTotalForce(delta, 'normalForces')';

%% Forces and torques
fo_muo = transforms.B_adjT_leg * [fo_raw';muo_raw'];
fc_muc = transforms.B_adjT_ankle * [fc_raw';muc_raw'];

if(strcmp(measurementType,'dualState')~=1)
   %% recomputing momment offsets to have zero initial angular acceleration if its not dualState
    muo = fo_muo(4:6,:) - mean(muo_calib,2)*ones(1,length(t));
    muc = fc_muc(4:6,:) - mean(muc_calib,2)*ones(1,length(t));
else
    %% leaving offsets as they are to have an initial compliance momment
    muo = fo_muo(4:6,:) + mu_calib_delta*ones(1,length(t));
    muc = fc_muc(4:6,:) - mu_calib_delta*ones(1,length(t));
    %muo = fo_muo(4:6,:);
    %muc = fc_muc(4:6,:);
end
    
fo = fo_muo(1:3,:) + f_calib_delta*ones(1,length(t));
fc = fc_muc(1:3,:)  - f_calib_delta*ones(1,length(t));

%% offset in skin to make it identical to FT measurements at calibrationtime
fc_zDelta = (mean(fc_z_calib - (fc_calib(3,:) + f_calib_delta(3)*ones(1,length(tCalib)))));
fc_z = fc_z - fc_zDelta;

%% IMU
%rotating accelerometer
a = (transforms.B_R_imu*a_raw');
%centering omega - removing the mean calibration omega
omegaCentered = (omega_raw')' - repmat(omegaOffset',size(omega_raw,1),1);
omega = (transforms.B_R_imu*omegaCentered')'.*(pi/180);

processedData.fo = fo;
processedData.muo = muo;
processedData.fc = fc;
processedData.muc = muc;
processedData.a = a;
processedData.omega = omega;
processedData.fc_z = fc_z;

   
%% plotting raw and corrected data
if(strcmp(plots,'makePlots') == 1)
    plotMeasurementData( t,rawData,processedData)
end

    
processedData = fakeMeasurement_completeLeg(processedData,fakeFlags,fakeValues,model,plots,t);

fo = processedData.fo;
muo = processedData.muo;
fc = processedData.fc;
muc = processedData.muc;
fo = processedData.fo;
a = processedData.a;
omega = processedData.omega;
fc_z = processedData.fc_z;
  
    tMeas = t;

    switch(measurementType)
        case 'withoutSkin'
            fprintf('\n setting up withoutSkin measurement\n');            
            model.x0 = [zeros(3,1);zeros(3,1);fo(:,1);muo(:,1);fc(:,1);muc(:,1);model.phi0];     
            yMeas = [a;omega';fo;muo;fc;muc]';
        case 'withSkin'
            fprintf('\n setting up withSkin measurement\n');            
            model.x0 = [zeros(3,1);zeros(3,1);fo(:,1);muo(:,1);fc(:,1);muc(:,1);model.phi0]; 
            yMeas = [a;omega';fo;muo;fc;muc;fc_z]';
        case 'dualState'
            fprintf('\n setting up dualState measurement\n');            
            K0 = eye(3)*model.kIni;
            model.x0 = [zeros(3,1);zeros(3,1);fo(:,1);muo(:,1);fc(:,1);muc(:,1);model.phi0;reshape(K0,9,1)]; 
            yMeas = [a;omega';fo;muo;fc;muc;fc_z]';
        otherwise
            disp('ERROR : Measurement Type Unknown');
    end
    
    

    %% testing forces and torques
    
    fprintf('Force test \n');
   
    fprintf('[  fc   fo   m*B_R_G*G_g fo+m*B_g-fc ] ');
    [fc(:,1) fo(:,1) model.m* euler2dcm(model.phi0)*model.G_g -fc(:,1)+fo(:,1)+model.m* euler2dcm(model.phi0)*model.G_g ]%          
    fprintf('Momment test\n');
    fprintf('[  muc   muo  ]');
    [muc(:,1) muo(:,1)]
    fprintf('Acceleration test \n');
    fprintf(' [ accl , gyrs] ');
    
    [a(1:3,1)  omega(1,1:3)'] 
    phiTrue = a_omega_raw(:,1:3)';
    disp('loaded measurement data');
    
    
    
    %model.m = 0.761;
    
    %model.I = [0.00253893, -4.51893e-6, -0.000903578;...
    %           -4.51893e-6,  0.00407487, 3.68679e-5;...
    %           -0.000903578, 3.68679e-5, 0.00208378];
           
            %ixx="0.00253893" ixy="-4.51893e-06" ixz="-0.000903578" iyy="0.00407487" iyz="3.68679e-05" izz="0.00208378
    
            

end

