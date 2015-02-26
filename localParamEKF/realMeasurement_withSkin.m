function [] = realMeasurement_withSkin(numberOfExperiment, whichLeg, whichSkin)
% REALMEASUREMENT_WITHSKIN loads data from the backward tipping experiments
%   including the feet skin to post-process these data.
%
%   REALMEASUREMENT_WITHSKIN() numberOfExperiments = 7, whichLeg = 'right',
%   whichSkin = 'right'
%   REALMEASUREMENT_WITHSKIN(numberOfExperiments, whichLeg, whichSkin)

disp('processing skin and other data');

if (nargin<1)
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

dtKalman = 0.01;

if(strcmp(leg_choice,'left')==1)
    leg_ft_offset = left_leg_ft_offset;
    foot_ft_offset = left_foot_ft_offset;
else
    leg_ft_offset = right_leg_ft_offset;
    foot_ft_offset = right_foot_ft_offset;
end
 
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

tMax = min([leg_ft.t(end),foot_ft.t(end),skin.t(end),inertial.t(end)]);
t = linspace(0,tMax,tMax/dtKalman);

f = interp1(foot_ft.t,foot_ft.f,t);
mu = interp1(foot_ft.t,foot_ft.mu,t);
del = interp1(skin.t,skin.data,t);
a_omega = interp1(inertial.t,inertial.data,t);

% KFoot = load('./skinFunctions/footStiffnessMatrix.mat');
% TFoot = load('./skinFunctions/Tmatrix.mat');

figure(1);
    subplot(2,1,1);
    plot(t,f);
    xlabel('time (sec)');
    ylabel('F/T Ankle Force (N)');
    axis tight;
    
    subplot(2,1,2);
    plot(t,mu);
    xlabel('time (sec)');
    ylabel('F/T Ankle Torque (Nm)');
    axis tight;
    
figure(2);
    plot(t,-computeTotalForce(del,'normalForces'));
    xlabel('time (sec)');
    ylabel('F_z from skin');
    
    axis tight;

figure(3);
    subplot(2,2,1);
    plot(t,a_omega(:,1:3));
    xlabel('time (sec)');
    ylabel('Euler Angles [deg]');
    legend('X', 'Y', 'Z');
    axis tight;
    
    subplot(2,2,2);
    plot(t,a_omega(:,4:6));
    xlabel('time (sec)');
    ylabel('linear acceleration [m/s^2]');
    legend('accX', 'accY', 'accZ');
    axis tight;
    
    subplot(2,2,3);
    plot(t,a_omega(:,7:9));
    xlabel('time (sec)');
    ylabel('angular speeed [deg/s]');
    legend('gyrX', 'gyrY', 'gyrZ');
    axis tight;
    
    subplot(2,2,4);
    plot(t,a_omega(:,10:12));
    xlabel('time (sec)');
    ylabel('magnetic field');
    legend('magX', 'magY', 'magZ');
    axis tight;
    
end