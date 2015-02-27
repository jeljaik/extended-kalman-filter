<<<<<<< HEAD
function [yMeas,tMeas,model] = realMeasurement_withSkin(dtKalman, model, plots, t_min, t_max, numberOfExperiment, whichLeg, whichSkin)
% REALMEASUREMENT_WITHSKIN loads data from the backward tipping experiments
%   including the feet skin to post-process these data.
%
%   REALMEASUREMENT_WITHSKIN() numberOfExperiments = 7, whichLeg = 'right',
%   whichSkin = 'right'
%   REALMEASUREMENT_WITHSKIN(numberOfExperiments, whichLeg, whichSkin)

disp('processing skin and other data');

if (nargin<6)
    numberOfExperiment = 7;
    whichLeg  = 'right';
    whichSkin = 'right';
end

%skin_data = importdata('./robotData/backwardTipping/dumperTippingSetup01/icub/skin/right_foot/data.log ');

%% TODO. 
% Find out the correspondence between the FT number in the
% wholeBodyDynamicsTree calibration dump and their corresponding leg/foot.
left_leg_ft_offset = zeros(1,6);
right_leg_ft_offset = zeros(1,6);
left_foot_ft_offset = zeros(1,6);
right_foot_ft_offset = zeros(1,6);

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
% leg to ankle
leg_p_ankle = [0.4776 0 0]' ;
%leg_T_ankle = [eye(3) leg_p_ankle ; zeros(3,1),1];
ankle_p_com = [0.024069 -0.000613931 0.0425846]';
%ankle_T_com = [eye(3) ankle_p_com ; zeros(3,1),1];

com_adj_ankle = [eye(3) zeros(3) ; -eye(3)*S(ankle_p_com) eye(3) ];
ankle_adj_leg = [eye(3) zeros(3) ; -eye(3)*S(leg_p_ankle) eye(3) ]; 
com_adj_leg = com_adj_ankle * ankle_adj_leg;

com_R_imu = [  0 -1 0 ;...
               0  0 1 ;...
              -1  0 0 ];

leg_ft.t = leg_ft_data(:,2)-leg_ft_data(1,2);
leg_ft.idx = leg_ft_data(:,1) - leg_ft_data(1,1);
leg_ft_data(:,3:8) = leg_ft_data(:,3:8) - repmat(leg_ft_offset,size(leg_ft_data,1),1);
left_leg_ft.f = leg_ft_data(:,3:5);
left_leg_ft.mu = leg_ft_data(:,6:8);

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

%% pre-processed interpolated data
fo_ = interp1(leg_ft.t,leg_ft.f,t);
muo_ = interp1(leg_ft.t,leg_ft.mu,t);
fc_ = interp1(foot_ft.t,foot_ft.f,t);
muc_ = interp1(foot_ft.t,foot_ft.mu,t);
delta_ = interp1(skin.t,skin.data,t);
a_omega = interp1(inertial.t,inertial.data,t);

%% IMU and skin
a_ = a_omega(:,4:6);
omega_ = a_omega(:,7:9);

del = (256 - del_)./255;
fc_x = computeTotalForce(delta_, 'normalForces')';

%% Forces and torques
fo_muo = com_adj_ankle * ankle_adj_leg * [fo_';muo_'];
fo = fo_muo(1:3,:);
muo = fo_muo(4:6,:);

fc_muc = com_adj_ankle * [fc_';muc_'];
fc = fc_muc(1:3,:);
muc = fc_muc(4:6,:);

%% Gyro
a = (-com_R_imu*a_');
omega = (-com_R_imu*omega_')*(pi/180);


    %% plotting raw and corrected data
    if(plots == 0)
        figure(1);
            subplot(2,2,1);
            plot(t,fo_);
            xlabel('time (sec)');
            ylabel('force (N)');
            legend('fX', 'fY', 'fZ');            
            axis tight;
            title('fo_{raw}');

            subplot(2,2,2);
            plot(t,muo_);
            xlabel('time (sec)');
            ylabel('torque (Nm)');
            legend('muX', 'muY', 'muZ');                                    
            axis tight;
            title('muo_{raw}');

            subplot(2,2,3);
            plot(t,fc_);
            xlabel('time (sec)');
            ylabel('force (N)');
            legend('fX', 'fY', 'fZ');                                    
            axis tight;
            title('fc_{raw}');

            subplot(2,2,4);
            plot(t,muc_);
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
            plot(t,fc_x);
            xlabel('time (sec)');
            ylabel('force (N)');
            legend('fX', 'fY', 'fZ');
            axis tight;

         figure(4);
            subplot(2,1,1);
            plot(t,a_);
            xlabel('time (sec)');
            ylabel('m/sec^2');
            legend('accX', 'accY', 'accZ');
            axis tight;
            title('Linear Acceleration a_{raw} [m/s^2]');

            subplot(2,1,2);
            plot(t,omega_);
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
    yMeas = [a;omega;fo;muo;fc;muc;fc_x]';
    tMeas = t;
    
    model.m = 0.761;
    
    model.I = [0.00253893, -4.51893e-6, -0.000903578;...
               -4.51893e-6,  0.00407487, 3.68679e-5;...
               -0.000903578, 3.68679e-5, 0.00208378];
           
            %ixx="0.00253893" ixy="-4.51893e-06" ixz="-0.000903578" iyy="0.00407487" iyz="3.68679e-05" izz="0.00208378
    
    model.x0 = [zeros(6,1);fo(:,1);muo(:,1);fc(:,1);muc(:,1);zeros(3,1)];

end
