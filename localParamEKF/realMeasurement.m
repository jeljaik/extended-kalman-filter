function [yMeas,tMeas, model] = realMeasurement(dtKalman,model,plots)
% ignore R, forceSim.
disp('Taking measurements from real data');
drawnow();

%% Key option
useFilteredAccln =0; %0 - false, 1- true;


%% offset of forces (from macumba output)
%FT0 = [75.594 -7.16053 230.176 -0.566831 -4.87001 0.192374]; %left arm
%FT1 = [131.083 -5.66526 94.2068 0.166921 -1.97958 -0.0276702]; %right arm
FT2 = [-6.80814 15.6911 -41.8068 -4.62421 1.63173 -0.898597];  %left leg
FT3 = [-15.1469 13.096 -44.1138 0.30103 -0.406419 -1.2199]; %left foot
FT4 = [-60.9461 -13.3928 22.4748 3.47944 -0.157395 0.247571]; %right leg 
FT5 = [36.2129 -57.6593 15.7812 0.450078 -2.05478 0.245422]; %right foot

delta_left_leg = FT2;
delta_left_foot = FT3;
delta_right_leg = FT4;
delta_right_foot = FT5;



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
left_leg_data(:,3:8) = left_leg_data(:,3:8) - repmat(delta_left_leg,size(left_leg_data,1),1);
left_leg.f = left_leg_data(:,3:5);
left_leg.mu = left_leg_data(:,6:8);

right_leg.t = right_leg_data(:,2) - right_leg_data(1,2);
right_leg.idx = right_leg_data(:,1) - right_leg_data(1,1);
right_leg_data(:,3:8) = right_leg_data(:,3:8) -repmat(delta_right_leg,size(right_leg_data,1),1);
right_leg.f = right_leg_data(:,3:5);
right_leg.mu = right_leg_data(:,6:8);

left_foot.t = left_foot_data(:,2) - left_foot_data(1,2);
left_foot.idx = left_foot_data(:,1) - left_foot_data(1,1);
left_foot_data(:,3:8) = left_foot_data(:,3:8) - repmat(delta_left_foot, size(left_foot_data,1),1);
left_foot.f = left_foot_data(:,3:5);
left_foot.mu = left_foot_data(:,6:8);

right_foot.t = right_foot_data(:,2) - right_foot_data(1,2);
right_foot.idx = right_foot_data(:,1) - right_foot_data(1,1);
right_foot_data(:,3:8) = right_foot_data(:,3:8) - repmat(delta_right_foot, size(right_foot_data,1),1);
right_foot.f = right_foot_data(:,3:5);
right_foot.mu = right_foot_data(:,6:8);


t_min = 53;
t_max = 63.5;
index_at_time = @(desTime,totalTime)find(totalTime>desTime,1,'first');

%% computing adjoint matrices of the transformation of leg and foot sensors to leg CoM
%Adj_o = [ eye(3) zeros(3) ; S([0 0 0.18102]') eye(3)];
%Adj_c = [ eye(3) zeros(3) ; S([0 0 -0.18102]') eye(3)];

if(plots==1)

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

end

%% resampling to uniform interval and averaging left and right leg components

tempArr = [left_leg.t(end) right_leg.t(end) left_foot.t(end) right_foot.t(end)];
endTime = min(tempArr);
tempDataPts = [left_leg.idx(end) right_leg.idx(end) left_foot.idx(end) right_foot.idx(end)];
dataPts = min(tempDataPts);
%deltaT = endTime./dataPts;

newTime = linspace(0,endTime,endTime./dtKalman);

fcDash_left = interp1(left_foot.t, left_foot.f,newTime);
mucDash_left = interp1(left_foot.t, left_foot.mu,newTime);

fcDash_right = interp1(right_foot.t, right_foot.f,newTime);
mucDash_right = interp1(right_foot.t, right_foot.mu,newTime);

foDash_left = interp1(left_leg.t, left_leg.f,newTime);
muoDash_left = interp1(left_leg.t, left_leg.mu,newTime);

foDash_right = interp1(right_leg.t, right_leg.f,newTime);
muoDash_right = interp1(right_leg.t, right_leg.mu,newTime);


fcDash = 0.5*(fcDash_left+fcDash_right);
mucDash = 0.5*(mucDash_left+mucDash_right);

foDash = 0.5*(foDash_left+foDash_right);
muoDash = 0.5*(muoDash_left+muoDash_right);


fc = fcDash'; % Due to first 3 rows of adjoint matrix above.
muc = S([0 0 -0.18102]') * fcDash' + mucDash';

fo = foDash'; % Due to first 3 rows of adjoint matrix above.
muo = S([0 0 0.18102]') * foDash' + muoDash';


minNewTime = index_at_time(t_min,newTime);
maxNewTime = index_at_time(t_max,newTime);

fc = fc(:,minNewTime:maxNewTime);
muc = muc(:,minNewTime:maxNewTime);
fo = fo(:,minNewTime:maxNewTime);
muo = muo(:,minNewTime:maxNewTime);
newTime = newTime(minNewTime:maxNewTime) - newTime(minNewTime);

if(plots==1)
figure(3);
subplot(2,2,1);
plot(newTime,fo); 
axis tight;
title('upper force (at leg CoM)');
subplot(2,2,2);
plot(newTime,muo);axis tight;
title('upper torque (at leg CoM)');
subplot(2,2,3);
plot(newTime,fc); axis tight;
title('lower force (at leg CoM)');
subplot(2,2,4);
plot(newTime,muc);axis tight;
title('lower torque (at leg CoM)');

figure(4);
subplot(1,2,1);
deltaf = fo-fc;
plot(newTime,deltaf(1,:),'r',newTime,deltaf(2,:),'b',newTime,deltaf(3,:),'g');
axis tight;
legend('F_x','F_y','F_z');
title('force difference');
subplot(1,2,2);
deltamu = muo-muc;
plot(newTime,muo-muc);
plot(newTime,deltamu(1,:),'r',newTime,deltamu(2,:),'b',newTime,deltamu(3,:),'g');
axis tight;
legend('\mu_x','\mu_y','\mu_z');
title('torque difference');

end
%% new model parameters from robot 
model.m = 4.9580;
model.x0 = [zeros(3,1);zeros(3,1);fo(1:3,1);fc(1:3,1);muo(1:3,1);muc(1:3,1);[0;0.01;0]]; %fake small phi rotation

[a ,a_filt] = extractViconData(dtKalman,plots);

minId = min(size(a,2),size(fo,2));

if(useFilteredAccln ==0) 
    yMeas = [a(:,1:minId);fo(:,1:minId);fc(:,1:minId);muo(:,1:minId);muc(:,1:minId)]';
else
    yMeas = [a_filt(:,1:minId);fo(:,1:minId);fc(:,1:minId);muo(:,1:minId);muc(:,1:minId)]';
end
    tMeas = newTime;%newTime(minNewTime:maxNewTime) - minNewTime;
end
