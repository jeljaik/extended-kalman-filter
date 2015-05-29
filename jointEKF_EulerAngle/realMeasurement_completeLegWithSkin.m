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

%skin_data = importdata('./robotData/backwardTipping/dumperTippingSetup01/icub/skin/right_foot/data.log ');
%% Params
   %% new model parameters from robot 
model.m = 4.9580;


%% obtaining mass-matrix from mexWBIModel (stored in utils)

load('./utils/MIni','M_at_q0');

%% TODO We need to extract foot's instead.
Mjts = M_at_q0(7:end,7:end);            % Leg mass matrix 
%MI_at_thigh = Mjts(14:16,14:16);

%%% checkup if this is for left leg or right leg. Not entirely sure if it will be different
%Tl = [0 1 0; 1 0 0 ; 0 0 -1];
%% Transformation of the mass matrix to COM reference frame
%Ic = Tl*MI_at_thigh*Tl' - S([0 0 -0.18102]')*S([0 0 -0.18102]')';
MI_at_thigh_r = Mjts(20:22,20:22);
 Tl_r = [0 -1 0; 1 0 0 ; 0 0 1];
 Ic = Tl_r*MI_at_thigh_r*Tl_r' - S([0 0 -0.18102]')*S([0 0 -0.18102]')';

% disp(Ic);
model.I = Ic;
%model.x0 = [zeros(3,1);omega(1,:)';fo(:,1);muo(:,1);fc(:,1);muc(:,1);zeros(3,1)];
model.gRot = [-1;0;0];
model.phi0 = [0,0.5*pi,0]';
model.K = zeros(3,3);
model.acclSign = -1;

%% OFFSETS. 
% Left leg
FT2 = [-4.11735 111.148 -131.104 -13.423 1.36924 -1.04304];
% LEft Foot
FT3 = [-52.0296 -5.64247 -18.0923 -0.516052 9.91345 0.717567];
% Right Leg
FT4 = [-5.65084 -10.5031 -24.6174 1.95779 2.61084 0.0036913];
% Right foot
FT5 = [-17.4992 -0.910681 18.0613 -0.824831 3.32657 -0.252851];

left_leg_ft_offset = FT2;
right_leg_ft_offset = FT4;
left_foot_ft_offset = FT3;
right_foot_ft_offset = FT5;

%%
expPath     = ['./robotData/backwardTipping/dumperTippingSetup0' ...
                num2str(numberOfExperiment) '/icub/'];
leg_choice  = whichLeg;
skin_choice = whichSkin;

% Leg F/T analog sensor
leg_ft_data   = importdata(strcat(expPath,leg_choice,'_leg/analog:o/data.log'));
% Foot F/T analog sensor
foot_ft_data  = importdata(strcat(expPath,leg_choice,'_foot/analog:o/data.log'));
% Foot skin data
skin_data     = importdata(strcat(expPath,'skin/',skin_choice,'_foot/data.log'));
% Inertial sensor attached to the foot (right)
inertial_data = importdata(strcat(expPath,'inertial/data.log'));


if(strcmp(leg_choice,'left')==1)
    leg_ft_offset = left_leg_ft_offset;
    foot_ft_offset = left_foot_ft_offset;
else
    leg_ft_offset = right_leg_ft_offset;
    foot_ft_offset = right_foot_ft_offset;
end
 
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

leg_p_com = [0 0 0.18102]';
ankle_p_com = [0 0 -0.18102]';

com_adjT_ankle = [eye(3) zeros(3) ; -eye(3)*S(ankle_p_com) eye(3) ];
com_adjT_leg = [eye(3) zeros(3) ; -eye(3)*S(leg_p_com) eye(3) ];

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
load('IMUOffset.mat','com_R_imu');

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

a_pre_calib = model.acclSign*a_omega_pre_calib(:,4:6);
omega_pre_calib = a_omega_pre_calib(:,7:9);

%omega_pre_calib = interp1(inertial.t,inertial.data(:,7:9),tCalib);
omegaOffset = mean((omega_pre_calib'),2);

%% Total Normal Force through the skin
% The following two lines can be replaced by totalForceFromSkinData() but
% this would read again the raw values. 
delta_proc_calib = dataPostProcessing(delta_pre_calib, 'normalForces');
fc_z_calib = computeTotalForce(delta_proc_calib, 'normalForces')';

%% Forces and torques
fo_muo_calib = com_adjT_leg * [fo_pre_calib';muo_pre_calib'];
fo_calib = fo_muo_calib(1:3,:);
muo_calib = fo_muo_calib(4:6,:);

fc_muc_calib = com_adjT_ankle * [fc_pre_calib';muc_pre_calib'];
fc_calib = fc_muc_calib(1:3,:);
muc_calib = fc_muc_calib(4:6,:);


%% Force offsets corrected to calibration dataset
f_calib_delta = (0.5)*( mean(-fo_calib+fc_calib,2) - model.m*9.81 * euler2dcm(model.phi0)*model.gRot);
mu_calib_delta = (0.5)*mean(-muo_calib + muc_calib,2);



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
a_pre_proc = model.acclSign*a_omega_pre_proc(:,4:6);
omega_pre_proc = a_omega_pre_proc(:,7:9);

%omegaCalib = interp1(inertial.t,inertial.data(:,7:9),tCalib);
%omegaOffset = mean(omega_calib,1);

%% Total Normal Force through the skin
% The following two lines can be replaced by totalForceFromSkinData() but
% this would read again the raw values. 
delta = dataPostProcessing(delta_pre_proc, 'normalForces');
fc_z = computeTotalForce(delta, 'normalForces')';

%% Forces and torques
fo_muo = com_adjT_leg * [fo_pre_proc';muo_pre_proc'];
fo = fo_muo(1:3,:) + f_calib_delta*ones(1,length(t));
muo = fo_muo(4:6,:) + mu_calib_delta*ones(1,length(t));

fc_muc = com_adjT_ankle * [fc_pre_proc';muc_pre_proc'];
fc = fc_muc(1:3,:)  - f_calib_delta*ones(1,length(t));
muc = fc_muc(4:6,:) - mu_calib_delta*ones(1,length(t));
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
a = (com_R_imu*a_pre_proc');
omegaCentered = (omega_pre_proc')' - repmat(omegaOffset',size(omega_pre_proc,1),1);
omega = (com_R_imu*omegaCentered')'.*(pi/180);



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
    yMeas = [a;omega';fo;muo;fc;muc;fc_z]';
    tMeas = t;
%a(:,1)
%a(:,1) - model.g*euler2dcm(model.phi0)*model.gRot
% % 
    PHI = kron(model.phi0.',eye(3));
    K0 = pinv(PHI) * muc(:,1);
    model.K = reshape(K0,3,3);
%        K0 = reshape(model.K,9,1);

    % model.x0 = [zeros(3,1);omega(1,:)';fo(:,1);muo(:,1);fc(:,1);muc(:,1);[0,0.5*pi,0]']; %% corresponds to -9.81 accln on Z
    model.x0 = [zeros(3,1);zeros(3,1);fo(:,1);muo(:,1);fc(:,1);muc(:,1);model.phi0;K0]; %% corresponds to -9.81 accln on Z

    %model.m = 0.761;
    
    %model.I = [0.00253893, -4.51893e-6, -0.000903578;...
    %           -4.51893e-6,  0.00407487, 3.68679e-5;...
    %           -0.000903578, 3.68679e-5, 0.00208378];
           
            %ixx="0.00253893" ixy="-4.51893e-06" ixz="-0.000903578" iyy="0.00407487" iyz="3.68679e-05" izz="0.00208378
    
            

end

