function comparePlots(experiment,whichDataset,trialNum,expID,processType,measurementType,plotDivide,legends)

idx = 1;
%the input arguments should be lists/vectors of same length
tf = isequal(size(experiment),size(whichDataset),size(trialNum),size(expID),size(processType),size(measurementType));

if(tf ~= true)
    fprintf('\n\ncomparePlots WARNING : All input arguments must be of same length for comparison\n');
end


numPlots = size(experiment,1);

for i = idx : numPlots
    if(strcmp(experiment{i},'foot') == 1)
        if(strcmp(whichDataset{i},'old') == 1)
            dataBaseFolder{i} = sprintf('./data/humanoids2015/foot/old/trial_%d/Exp_%d/',trialNum(i),expID(i));
        end
        if(strcmp(whichDataset{i},'new') == 1)
            dataBaseFolder{i} = sprintf('./data/humanoids2015/foot/new/trial_%d/Exp_%d/',trialNum(i),expID(i));
        end
    end
    if(strcmp(experiment{i},'leg') == 1)
        if(strcmp(whichDataset{i},'old') == 1)
            dataBaseFolder{i} = sprintf('./data/humanoids2015/leg/old/trial_%d/Exp_%d/',trialNum(i),expID(i));
        end
        if(strcmp(whichDataset{i},'new') == 1)
            dataBaseFolder{i} = sprintf('./data/humanoids2015/leg/new/trial_%d/Exp_%d/',trialNum(i),expID(i));
        end
    end
    load(strcat(dataBaseFolder{i},'filteredResult.mat'));
    tK{i} = tKalman;
    yM{i} = yMeas;
    source{i} = 2;
    XUpt{i} = Xupdt;
    XPred{i} = Xhat;
    tMin{i} = tK{i}(1);

end

figure();
for i = idx : numPlots
if(strcmp(processType{i},'withoutCompliance')~=1)
    stateVar = 22:30;
    K = XUpt{i}(:,stateVar);
    K_norm = (K(:,1).^2 + K(:,2).^2 + K(:,3).^2 + K(:,4).^2 + K(:,5).^2 + K(:,6).^2 + K(:,7).^2 + K(:,8).^2 + K(:,9).^2 ).^0.5;
    
    plot(tK{i}(:,idx:end),K_norm(idx:end,:));
    hold on    
end
end
title('Stiffness norm evolution');
xlabel('Time (s)');
ylabel('|K| Nm/rad');
legend(legends');

% % 
stateVar = 19:21;
numSubFig=length(stateVar);

  
subFig = [numSubFig,1];

for i = 1:numSubFig
        subplot(subFig(1),subFig(2),i);
        
     for j = idx : numPlots  
         theta = rad2deg(XUpt{j}(:,stateVar));
%          theta = XUpt{j}(:,stateVar);
        plot(tK{i}(:,idx:end),theta(idx:end,i));
        xlabel('Time (s)');
        ylabel('E_\phi degs');
        hold on    
     end    
    title(strcat('Expection of orientation: ',measurementType{i}));
    legend(legends');
end
end

 
% if(strcmp(plotDivide,'true')==1)
%     steadyStateTime = 6.0050;
%     steadyStateIdx = find(abs(tK(i) - steadyStateTime) < 0.001);
% end
