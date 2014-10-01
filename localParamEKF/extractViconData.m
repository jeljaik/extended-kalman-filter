function [a, a_filt] = extractViconData(deltaTKalman,plots)

%% loading from dataset

dataset = importdata('./real_data/trial_03_vicon.csv');

%dataset time
t = [0;cumsum((1/120)*ones(size(dataset,1)-1,1))];%0:1/120:size(dataset,1);


index_at_time = @(desTime,totalTime)find(totalTime>desTime,1,'first');
tMin = 47.0;
tMax = 57.5;
idxMin = index_at_time(tMin,t);
idxMax = index_at_time(tMax,t);

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

%legVel = diff(leg2,1,1)./repmat(diff(t),1,3);
legAccl = diff(leg2,2,1)./repmat(diff(t(1:end-1)),1,3);

%windowSize = 50;
%torsoVelDash = sgolayfilt(torsoVel,5,101);%filter(ones(1,windowSize)/windowSize,1,torsoVel); %(1/3)*([torsoVel(1:end-1,:); zeros(1,3)]+ torsoVel + [zeros(1,3) ; torsoVel(2:end,:)]);
legAcclDash = sgolayfilt(legAccl,5,101);
%diff(torsoVelDash)./repmat(diff(t(1:end-1),1,1),1,3);

if(plots==1)
    figure;
    plot(t,leg2);
    xlabel('time t(sec)');
    ylabel('position of Acclerometer in global frame(m)');
    % 
    % figure(2);
    % plot(t(1:end-1),torsoVel);
    % 
    % figure(3)
    % plot(t(1:end-1), torsoVelDash);
    % 
    figure;
    plot(t(idxMin:idxMax),legAccl(idxMin:idxMax,:)); axis tight;
    xlabel('time t(sec)');
    ylabel('accln');

    figure;
    plot(t(idxMin:idxMax), legAcclDash(idxMin:idxMax,:)); axis tight;
    xlabel('time t(sec)');
    ylabel('accln smoothed');
    % 
    % %end
    end

if(isempty(deltaTKalman))
   newTime = linspace(0,t(end),round(length(t)/10));
else
   newTime = linspace(0,t(end),t(end)/deltaTKalman);
end
 a_F = interp1(t(1:end-2),legAccl,newTime,'pchirp');
 a_filt_F = interp1(t(1:end-2),legAcclDash,newTime,'pchirp');
 
 idMin = index_at_time(tMin,newTime);
 idMax = index_at_time(tMax,newTime);
 
 a = a_F(idMin:idMax,:)';
 a_filt = a_filt_F(idMin:idMax,:)';

end