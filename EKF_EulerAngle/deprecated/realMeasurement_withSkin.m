function [yMeas,tMeas,model,R] = realMeasurement_withSkin(dtKalman, model, plots, t_min, t_max, numberOfExperiment, whichLeg, whichSkin)
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

 [leg_ft_data,foot_ft_data,skin_data, inertial_data] = prepareRealData();
 [transformations] = obtainTransformations();
 
%% Rototranslation definitions
% leg to ankle
leg_p_ankle = [0 0 0.4776]';%[0.4776 0 0]' ;
%leg_T_ankle = [eye(3) leg_p_ankle ; zeros(3,1),1];
ankle_p_com = [0.024069 -0.000613931 0.0425846]';
%ankle_T_com = [eye(3) ankle_p_com ; zeros(3,1),1];

com_adj_ankle = [eye(3) zeros(3) ; -eye(3)*S(ankle_p_com) eye(3) ];
ankle_adj_leg = [eye(3) zeros(3) ; -eye(3)*S(leg_p_ankle) eye(3) ]; 
com_adj_leg = com_adj_ankle * ankle_adj_leg;
% 
% com_R_imu = [  -1  0   0 ;...
%                0   0  -1 ;...
%                0   1   0 ];

com_R_imu = [  1  0   0 ;...
               0   0  1 ;...
               0   -1   0 ];
leg_ft.t = leg_ft_data(:,2)-leg_ft_data(1,2);
leg_ft.idx = leg_ft_data(:,1) - leg_ft_data(1,1);
leg_ft_data(:,3:8) = leg_ft_data(:,3:8) - repmat(leg_ft_offset,size(leg_ft_data,1),1);
leg_ft.f = leg_ft_data(:,3:5);
leg_ft.mu = leg_ft_data(:,6:8);

foot_ft.t = foot_ft_data(:,2)-foot_ft_data(1,2);
foot_ft.idx = foot_ft_data(:,1) - foot_ft_data(1,1);
foot_ft_data(:,3:8) = foot_ft_data(:,3:8) - repmat(foot_ft_offset,size(foot_ft_data,1),1);
foot_ft.f = foot_ft_data(:,3:5);
foot_ft.mu = foot_ft_data(:,6:8);

skin.t = skin_data(:,2) - skin_data(1,2);
skin.idx = skin_data(:,1) - skin_data(1,1);
skin.data = skin_data(:,3:end);

inertial.t = inertial_data(:,2)-inertial_data(1,2);
inertial.idx = inertial_data(:,1) - inertial_data(1,1);
inertial.data = inertial_data(:,3:end);

tMax = min([leg_ft.t(end),foot_ft.t(end),skin.t(end),inertial.t(end),t_max]);
t = linspace(t_min,tMax,(tMax - t_min)/dtKalman);

tCalib = linspace(0,t_min,(t_min)/dtKalman);


%% calibration and covariance estimation data
fo_pre_calib = interp1(leg_ft.t,leg_ft.f,tCalib);
muo_pre_calib = interp1(leg_ft.t,leg_ft.mu,tCalib);
fc_pre_calib = interp1(foot_ft.t,foot_ft.f,tCalib);
muc_pre_calib = interp1(foot_ft.t,foot_ft.mu,tCalib);
delta_pre_calib = interp1(skin.t,skin.data,tCalib);
a_omega_pre_calib = interp1(inertial.t,inertial.data,tCalib);

a_pre_calib = -a_omega_pre_calib(:,4:6);
omega_pre_calib = a_omega_pre_calib(:,7:9);

%omega_pre_calib = interp1(inertial.t,inertial.data(:,7:9),tCalib);
omegaOffset = mean((com_R_imu*omega_pre_calib'),2);

%% Total Normal Force through the skin
% The following two lines can be replaced by totalForceFromSkinData() but
% this would read again the raw values. 
delta_proc_calib = dataPostProcessing(delta_pre_calib, 'normalForces');
fc_z_calib = computeTotalForce(delta_proc_calib, 'normalForces')';

%% Forces and torques
fo_muo_calib = com_adj_leg * [fo_pre_calib';muo_pre_calib'];
fo_calib = fo_muo_calib(1:3,:);
muo_calib = fo_muo_calib(4:6,:);

fc_muc_calib = com_adj_ankle * [fc_pre_calib';muc_pre_calib'];
fc_calib = fc_muc_calib(1:3,:);
muc_calib = fc_muc_calib(4:6,:);

%% IMU
a_calib = (com_R_imu*a_pre_calib');
%omegaCentered = omega_calib - repmat(omegaOffset,size(omega_pre_proc,1),1);
omega_calib = (com_R_imu*omega_pre_calib')*(pi/180);

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
a_pre_proc = -a_omega_pre_proc(:,4:6);
omega_pre_proc = a_omega_pre_proc(:,7:9);

%omegaCalib = interp1(inertial.t,inertial.data(:,7:9),tCalib);
%omegaOffset = mean(omega_calib,1);

%% Total Normal Force through the skin
% The following two lines can be replaced by totalForceFromSkinData() but
% this would read again the raw values. 
delta = dataPostProcessing(delta_pre_proc, 'normalForces');
fc_z = computeTotalForce(delta, 'normalForces')';

%% Forces and torques
fo_muo = com_adj_leg * [fo_pre_proc';muo_pre_proc'];
fo = fo_muo(1:3,:);
muo = fo_muo(4:6,:);

fc_muc = com_adj_ankle * [fc_pre_proc';muc_pre_proc'];
fc = fc_muc(1:3,:);
muc = fc_muc(4:6,:);

%% IMU
a = (com_R_imu*a_pre_proc');
omegaCentered = (com_R_imu*omega_pre_proc')' - repmat(omegaOffset',size(omega_pre_proc,1),1);
omega = (omegaCentered).*(pi/180);



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
    end
  %  idx = t>t_min;
  %  yMeas = [a(:,idx);omega(:,idx);fo(:,idx);muo(:,idx);fc(:,idx);muc(:,idx);fc_x(:,idx)]';
    yMeas = [a;omega';fo;muo;fc;muc;fc_z]';
    tMeas = t;
    
    model.m = 0.761;
    
    model.I = [0.00253893, -4.51893e-6, -0.000903578;...
               -4.51893e-6,  0.00407487, 3.68679e-5;...
               -0.000903578, 3.68679e-5, 0.00208378];
           
            %ixx="0.00253893" ixy="-4.51893e-06" ixz="-0.000903578" iyy="0.00407487" iyz="3.68679e-05" izz="0.00208378
    
    model.x0 = [zeros(3,1);omega(1,:)';fo(:,1);muo(:,1);fc(:,1);muc(:,1);model.phi0];

end
