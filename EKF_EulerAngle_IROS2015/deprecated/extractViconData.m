function [a, a_filt] = extractViconData(deltaTKalman,plots,tMinD,tMaxD)

%% vicon data tuning parameters
offset = 6.15;
delta_phiZ = pi*0.9; % in range 
delta_thetaX = pi*1.525; %ZXZ convention


RZ = @(phi)[cos(phi) -sin(phi) 0; sin(phi) cos(phi) 0; 0 0 1];
RX = @(theta)[1 0 0; 0 cos(theta) -sin(theta); 0 sin(theta) cos(theta)];
%in this problem RZPrime is eye(3)

RWB = RZ(pi - delta_phiZ)*RX(pi - delta_thetaX); 


%% loading from dataset

dataset = importdata('./real_data/trial_03_vicon.csv');

%dataset time
t = [0;cumsum((1/120)*ones(size(dataset,1)-1,1))];%0:1/120:size(dataset,1);


index_at_time = @(desTime,totalTime)find(totalTime>desTime,1,'first');

if(tMinD~=tMaxD)
    tMin = tMinD - offset;
    tMax = tMaxD - offset;
    
    idxMin = index_at_time(tMin,t);
    idxMax = index_at_time(tMax,t);
else
    tMin = t(1);
    tMax = t(end);
    idxMin = 1;
    idxMax = length(t)-2;
end
%tMin = 50;%47.0;
%tMax = 57.5;



%% ignoring additional data from vicon dataset
%leg1 = dataset(:,2:4)./100;
leg2 = dataset(:,5:7)./100;
%leg3 = dataset(:,8:10)./100;

%leg2 = interp1(t,dataset(:,5:7),newTime);

%t = newTime';
% 
% waist1 = dataset(:,11:13)./100;
% waist2 = dataset(:,12:14)./100;
% waist3 = dataset(:,15:17)./100;

legVel = diff(leg2,1,1)./repmat(diff(t),1,3);
legAccl = diff(legVel,1,1)./repmat(diff(t(1:end-1)),1,3);
%legAccl = diff(leg2,1,2)./repmat(diff(t(1:end-1)),1,3);

%windowSize = 50;
%torsoVelDash = sgolayfilt(torsoVel,5,101);%filter(ones(1,windowSize)/windowSize,1,torsoVel); %(1/3)*([torsoVel(1:end-1,:); zeros(1,3)]+ torsoVel + [zeros(1,3) ; torsoVel(2:end,:)]);
legAcclDash = sgolayfilt(legAccl,5,101);
%diff(torsoVelDash)./repmat(diff(t(1:end-1),1,1),1,3);

%legAccl(:,3) = -legAccl(:,3);
%legAcclDash(:,3) = -legAcclDash(:,3);

if(plots==1)
    figure;
    plot(t,leg2);
    xlabel('offset time t(sec)');
    ylabel('position of Accelerometer in global frame(m)');
    % 
    % figure(2);
    % plot(t(1:end-1),torsoVel);
    % 
    % figure(3)
    % plot(t(1:end-1), torsoVelDash);
    % 
%     figure;
%     plot(t(idxMin:idxMax),legAccl(idxMin:idxMax,:)); axis tight;
%     xlabel('time t(sec)');
%     ylabel('accln unrotated');

    figure;
    plot(t(idxMin:idxMax)+offset, legAcclDash(idxMin:idxMax,:)); axis tight;
    xlabel('offset time t(sec)');
    ylabel('accln unrotated smoothed');
    legend('a_x','a_y','a_z');
    title('Original');
    % 
    % %end
    
%     figure;
%     plot(t(idxMin:idxMax),(RWB*legAccl(idxMin:idxMax,:)')'); axis tight;
%     xlabel('time t(sec)');
%     ylabel('accln rotated');

    figure;
    plot(t(idxMin:idxMax)+offset, (RWB*legAcclDash(idxMin:idxMax,:)')'); axis tight;
    xlabel('offset time t(sec)');
    ylabel('accln rotated smoothed');
    legend('a_x','a_y','a_z');
    title('Rotated');
    end

if(isempty(deltaTKalman))
   %newTime = [t(idxMin)+offset:round(length(t)/10):t(idxMax)+offset]';
   ntMin = t(idxMin)+offset;
   ntMax = t(idxMax)+offset;
   newTime = linspace(ntMin,ntMax,(ntMax-ntMin)/10+1);
else
%   newTime = [t(idxMin)+offset:deltaTKalman:t(idxMax)+offset]';
   ntMin = t(idxMin)+offset;
   ntMax = t(idxMax)+offset;
   newTime = linspace(ntMin,ntMax,(ntMax-ntMin)/deltaTKalman+1);

   %linspace(0,t(end),t(end)/deltaTKalman);
end
 a_F = interp1(t(idxMin:idxMax)+offset,(RWB*legAccl(idxMin:idxMax,:)')',newTime);
 a_filt_F = interp1(t(idxMin:idxMax)+offset,(RWB*legAcclDash(idxMin:idxMax,:)')',newTime);
 
 %idMin = index_at_time(tMin,newTime);
 %idMax = index_at_time(tMax,newTime);
 
 a = a_F';%(idMin:idMax,:)';
 %a(3,:) = -a(3,:);
 a_filt = a_filt_F';%(idMin:idMax,:)';
 %a_filt(3,:) = -a_filt(3,:);   
end