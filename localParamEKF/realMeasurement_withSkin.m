disp('processing skin and other data');

%skin_data = importdata('./robotData/backwardTipping/dumperTippingSetup01/icub/skin/right_foot/data.log ');

left_leg_ft_offset = zeros(1,6);
right_leg_ft_offset = zeros(1,6);
left_foot_ft_offset = zeros(1,6);
right_foot_ft_offset = zeros(1,6);

expPath = './robotData/backwardTipping/dumperTippingSetup01/icub/';
leg_choice='left';
skin_choice='right';

leg_ft_data = importdata(strcat(expPath,leg_choice,'_leg/analog:o/data.log'));
foot_ft_data = importdata(strcat(expPath,leg_choice,'_foot/analog:o/data.log'));
skin_data =  importdata(strcat(expPath,'skin/',skin_choice,'_foot/data.log'));
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

KFoot = load('./skinFunctions/footStiffnessMatrix.mat');
TFoot = load('./skinFunctions/Tmatrix.mat');

figure(1);
    subplot(2,1,1);
    plot(t,f);
    xlabel('time (sec)');
    ylabel('force (N)');
    axis tight;
    
    subplot(2,1,2);
    plot(t,mu);
    xlabel('time (sec)');
    ylabel('torque (Nm)');
    axis tight;
    
figure(2);
    plot(t,(TFoot.ans'*del')');
    xlabel('time (sec)');
    axis tight;

figure(3);
    subplot(2,2,1);
    plot(t,a_omega(:,1:3));
    xlabel('time (sec)');
    axis tight;
    
    subplot(2,2,2);
    plot(t,a_omega(:,4:6));
    xlabel('time (sec)');
    axis tight;
    
    subplot(2,2,3);
    plot(t,a_omega(:,7:9));
    xlabel('time (sec)');
    axis tight;
    
    subplot(2,2,4);
    plot(t,a_omega(:,10:12));
    xlabel('time (sec)');
    axis tight;