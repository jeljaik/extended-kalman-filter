
function [yMeas,tMeas,model,R] = realMeasurement_completeLegWithSkin(dtKalman, model, plots, t_min, t_max, numberOfExperiment, whichLeg, whichSkin)
% REALMEASUREMENT_WITHSKIN loads data from the backward tipping experiments
%   including the feet skin to post-process these data.
%
%   REALMEASUREMENT_WITHSKIN(dtKalman, model, plots, t_min, t_max) When 
%   called like this, numberOfExperiments = 7, whichLeg = 'right',
%   whichSkin = 'right'.
%   REALMEASUREMENT_WITHSKIN(numberOfExperiments, whichLeg, whichSkin)

disp('processing skin and other data');

if(nargin<1)
    dtKalman = 0.01;
    model = struct();
    plots = 0;
    t_min = 0;
    t_max = inf;
end

if (nargin<6)
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
 [model,tMax,leg_ft,foot_ft,skin,inertial,transforms] = modelSensorParams_completeLeg(whichLeg,whichSkin,numberOfExperiment,t_max);

 model.dtKalman = dtKalman;
%% Rototranslation definitions
% % leg to ankle
% leg_p_ankle = [0 0 0.4776]';%[0.4776 0 0]' ;
% %leg_T_ankle = [eye(3) leg_p_ankle ; zeros(3,1),1];
% ankle_p_com = [0.024069 -0.000613931 0.0425846]';
% %ankle_T_com = [eye(3) ankle_p_com ; zeros(3,1),1];
% 
% com_adj_ankle = [eye(3) zeros(3) ; -eye(3)*S(ankle_p_com) eye(3) ];
% ankle_adj_leg = [eye(3) zeros(3) ; -eye(3)*S(leg_p_ankle) eye(3) ]; 
% com_adj_leg = com_adj_ankle * ankle_adj_leg;

% 
% com_R_imu = [  -1  0   0 ;...
%                0   0  -1 ;...
%                0   1   0 ];

% com_R_imu = [  1    0   0 ;...
%                0    0   1 ;...
%                0   -1   0 ];

%imuDel1 = 0.25*pi;%0.5*pi-0.35*pi;%-0.1*pi
%imuDel2 = 0.0;%0.01*pi;%0.005*pi;
%com_R_imu = euler2dcm([pi/2+imuDel1,pi/2+imuDel2,-pi/2])
t = linspace(t_min,tMax,(tMax - t_min)/dtKalman);

tCalib = linspace(0,t_min,(t_min)/dtKalman);


%% calibration and covariance estimation data
fo_pre_calib = interp1(leg_ft.t,leg_ft.f,tCalib);
muo_pre_calib = interp1(leg_ft.t,leg_ft.mu,tCalib);
fc_pre_calib = interp1(foot_ft.t,foot_ft.f,tCalib);
muc_pre_calib = interp1(foot_ft.t,foot_ft.mu,tCalib);
delta_pre_calib = interp1(skin.t,skin.data,tCalib);
a_omega_pre_calib = interp1(inertial.t,inertial.data,tCalib);

a_pre_calib = acclSign*a_omega_pre_calib(:,4:6);
omega_pre_calib = a_omega_pre_calib(:,7:9);

%omega_pre_calib = interp1(inertial.t,inertial.data(:,7:9),tCalib);
omegaOffset = mean((omega_pre_calib'),2);

%% Total Normal Force through the skin
% The following two lines can be replaced by totalForceFromSkinData() but
% this would read again the raw values. 
delta_proc_calib = dataPostProcessing(delta_pre_calib, 'normalForces');
fc_z_calib = computeTotalForce(delta_proc_calib, 'normalForces')';

%% Forces and torques
fo_muo_calib = transforms.B_adjT_leg * [fo_pre_calib';muo_pre_calib'];
fo_calib = fo_muo_calib(1:3,:);
muo_calib = fo_muo_calib(4:6,:);

fc_muc_calib = transforms.B_adjT_ankle * [fc_pre_calib';muc_pre_calib'];
fc_calib = fc_muc_calib(1:3,:);
muc_calib = fc_muc_calib(4:6,:);


%% Force offsets corrected to calibration dataset
f_calib_delta = (0.5)*( mean(+fc_calib-fo_calib,2) - model.m* euler2dcm(model.phi0)*model.G_g);
mu_calib_delta = (0.5)*mean(+muc_calib - muo_calib,2);

%f_calib_delta = zeros(size(f_calib_delta));
%mu_calib_delta = zeros(size(mu_calib_delta));

%% IMU
a_calib = (transforms.B_R_imu*a_pre_calib');
%omegaCentered = omega_calib - repmat(omegaOffset,size(omega_pre_proc,1),1);
omega_calib = (transforms.B_R_imu*omega_pre_calib')*(pi/180);
% 
% if(strcmp(fakeIMU,'true') == 1)
%     aPerfect = fakeAccl*ones(1,size(a_calib,2));
%     omegaPerfect= zeros(size(omega_calib));
%     a_calib = aPerfect;
%     omega_calib = omegaPerfect;
% end


yCalib = [a_calib;omega_calib;fo_calib;muo_calib;fc_calib;muc_calib;fc_z_calib]';

covMat = cov(yCalib);
R = covMat;


%% pre-processed interpolated data
fo_pre_proc = interp1(leg_ft.t,leg_ft.f,t);
muo_pre_proc = interp1(leg_ft.t,leg_ft.mu,t);
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
fo_muo = transforms.B_adjT_leg * [fo_pre_proc';muo_pre_proc'];
fo = fo_muo(1:3,:) + f_calib_delta*ones(1,length(t));
%muo = fo_muo(4:6,:) + mu_calib_delta*ones(1,length(t));
muo = fo_muo(4:6,:); %- mean(muo_calib,2)*ones(1,length(t));

fc_muc = transforms.B_adjT_ankle * [fc_pre_proc';muc_pre_proc'];
fc = fc_muc(1:3,:)  - f_calib_delta*ones(1,length(t));
%muc = fc_muc(4:6,:) - mu_calib_delta*ones(1,length(t));
muc = fc_muc(4:6,:); %- mean(muc_calib,2)*ones(1,length(t));


% 
% fo_muo = com_adjT_leg * [fo_pre_proc';muo_pre_proc'];
% fo = fo_muo(1:3,:);
% muo = fo_muo(4:6,:);
% 
% fc_muc = com_adjT_ankle * [fc_pre_proc';muc_pre_proc'];
% fc = fc_muc(1:3,:);
% muc = fc_muc(4:6,:);

%% offset in skin to make it identical to FT measurements at calibrationtime
fc_zDelta = (mean(fc_z_calib - (fc_calib(3,:) - f_calib_delta(3)*ones(1,length(tCalib)))));
fc_z = fc_z - fc_zDelta;
%% IMU
a = (transforms.B_R_imu*a_pre_proc');
omegaCentered = (omega_pre_proc')' - repmat(omegaOffset',size(omega_pre_proc,1),1);
omega = (transforms.B_R_imu*omegaCentered')'.*(pi/180);


    %% plotting raw and corrected data
    if(plots == 0)
        figure(1);
            % 
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
        
        if(plots == 0)
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
       
       %fcPerfect =  model.m.*model.B0_g * ones(1,size(fc,2));

       fczPerfect = fcPerfect(3,:);
       
       
       if(plots == 0)
        
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

  yMeas = [a;omega';fo;muo;fc;muc;fc_z]';
    tMeas = t;
    
    
    
%     Initialise K
%  
%     PHI = kron(model.phi0.',eye(3));
%     K0 = pinv(PHI)*(muo(:,1) - muc(:,1));
    K0 = ones(9,1);
    model.K = reshape(K0,3,3);
    - muc(:,1) + muo(:,1)
    
%a(:,1)
%a(:,1) - model.g*euler2dcm(model.phi0)*model.gRot
    % model.x0 = [zeros(3,1);omega(1,:)';fo(:,1);muo(:,1);fc(:,1);muc(:,1);[0,0.5*pi,0]']; %% corresponds to -9.81 accln on Z
    model.x0 = [zeros(3,1);zeros(3,1);fo(:,1);muo(:,1);fc(:,1);muc(:,1);model.phi0;K0]; %% corresponds to -9.81 accln on Z

    %% testing forces and torques
    
    fprintf('Force test \n');
   
    fprintf('[  fc   fo   m*B_R_G*G_g fo+m*B_g-fc ] ');
    [fc(:,1) fo(:,1) model.m* euler2dcm(model.phi0)*model.G_g -fc(:,1)+fo(:,1)+model.m* euler2dcm(model.phi0)*model.G_g ]
    
    
    fprintf('Momment test\n');
    fprintf('[  muc   muo  ]');
    [muc(:,1) muo(:,1)]
    fprintf('Acceleration test \n');
    fprintf(' [ accl , gyrs] ');
    
    [a(1:3,1)  omega(1,1:3)'] 
    
    disp('loaded measurement data');
    
    
    
    %model.m = 0.761;
    
    %model.I = [0.00253893, -4.51893e-6, -0.000903578;...
    %           -4.51893e-6,  0.00407487, 3.68679e-5;...
    %           -0.000903578, 3.68679e-5, 0.00208378];
           
            %ixx="0.00253893" ixy="-4.51893e-06" ixz="-0.000903578" iyy="0.00407487" iyz="3.68679e-05" izz="0.00208378
    
            

end

