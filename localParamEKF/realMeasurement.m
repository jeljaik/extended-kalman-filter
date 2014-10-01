%function [yMeasSim,model] = realMeasurement(tKalman,R,model,forceSim,plots)

%% loading data from dataset store

left_leg_data = importdata('./real_data/iniTrialDir3/left_leg_ft/data.log');
right_leg_data = importdata('./real_data/iniTrialDir3/left_leg_ft/data.log');
left_foot_data = importdata('./real_data/iniTrialDir3/left_foot_ft/data.log');
right_foot_data = importdata('./real_data/iniTrialDir3/right_foot_ft/data.log');
%left_foot_skin= importdata('./real_data/iniTrialDir3/left_foot_skin/data.log');
%right_foot_skin= importdata('./real_data/iniTrialDir3/left_foot_skin/data.log');

%% visualize raw data
left_leg.t = left_leg_data(:,2) - left_leg_data(1,2);
left_leg.idx = left_leg_data(:,1) - left_leg_data(1,1);
left_leg.f = left_leg_data(:,3:5);
left_leg.mu = left_leg_data(:,6:8);

right_leg.t = right_leg_data(:,2) - right_leg_data(1,2);
right_leg.idx = right_leg_data(:,1) - right_leg_data(1,1);
right_leg.f = right_leg_data(:,3:5);
right_leg.mu = right_leg_data(:,6:8);

left_foot.t = left_foot_data(:,2) - left_foot_data(1,2);
left_foot.idx = left_foot_data(:,1) - left_foot_data(1,1);
left_foot.f = left_foot_data(:,3:5);
left_foot.mu = left_foot_data(:,6:8);

right_foot.t = right_foot_data(:,2) - right_foot_data(1,2);
right_foot.idx = right_foot_data(:,1) - right_foot_data(1,1);
right_foot.f = right_foot_data(:,3:5);
right_foot.mu = right_foot_data(:,6:8);


t_min = 53;
t_max = 63;
index_at_time = @(desTime,totalTime)find(totalTime>desTime,1,'first');




figure(1);
subplot(2,2,1);
plot(left_leg.t,left_leg.f); 
axis tight;
title('left leg force - raw data');
subplot(2,2,2);
plot(right_leg.t,right_leg.f);axis tight;
title('right leg force - raw data');
subplot(2,2,3);
plot(left_foot.t,left_foot.f); axis tight;
title('left foot force - raw data');
subplot(2,2,4);
plot(right_foot.t,right_foot.f);axis tight;
title('right foot force - raw data');

figure(2);
subplot(2,2,1);
plot(left_leg.t,left_leg.mu); axis tight;
title('left leg torque - raw data');
subplot(2,2,2);
plot(right_leg.t,right_leg.mu);axis tight;
title('right leg torque - raw data');
subplot(2,2,3);
plot(left_foot.t,left_foot.mu); axis tight;
title('left foot torque - raw data');
subplot(2,2,4);
plot(right_foot.t,right_foot.mu);axis tight;
title('right foot torque - raw data');
%end



