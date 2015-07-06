clc;
display('plotting comparison plots');
close all;
clear all;
% Loading compliance results
dataBaseFolder = cell(3);

numOfExperiments = 1:3; %number of experiments
% 1 -  no stiffness and no skin
% 2 -  no stiffness and including skin
% 3 - including stiffness and including skin

for expID = 1:3
	dataBaseFolder{expID}= sprintf('./data/humanoids2015/Exp_%d/',expID);
    load(strcat(dataBaseFolder{expID},'filteredResult.mat'));
    tK{expID} = tKalman;
    yM{expID} = yMeas;
    source{expID} = 2;
    XUpt{expID} = Xupdt;
    XPred{expID} = Xhat;
    tMin{expID} = tK{expID}(1);

end
steadyStateTime = 6.0050;
steadyStateIdx = find(abs(tK{3} - steadyStateTime) < 0.001);

idx = 1; 
stateVar = 22:30;
K = XUpt{3}(:,stateVar);
K_norm = (K(:,1).^2 + K(:,2).^2 + K(:,3).^2 + K(:,4).^2 + K(:,5).^2 + K(:,6).^2 + K(:,7).^2 + K(:,8).^2 + K(:,9).^2 ).^0.5;
figure();
plot(tK{3}(idx:end),K_norm(idx:end,:));
title('Stiffness norm evolution');
xlabel('Time (s)');
ylabel('|K| Nm/rad');

pT.xlabelText = {'Time t(sec)',...
                 'Time t(sec)',...
                 'Time t(sec)'
};
% % orientation
stateVar = 19:21;
X_des = [0 0.5*pi 0];

pT.titleText = {'Expectation of Orientation steadystate',...
                '',...
                ''              
};
pT.ylabelText = {'E(\phi_x) degs',...
                 'E(\phi_y) degs',...
                 'E(\phi_z) degs'
};

cols = {'b','g','r'};
comparisonPlotFilterResultTimeSeries( tK{1}(idx:steadyStateIdx), rad2deg(XUpt{1}(idx:steadyStateIdx,:)),rad2deg(XUpt{2}(idx:steadyStateIdx,:)),rad2deg(XUpt{3}(idx:steadyStateIdx,:)), stateVar, pT,cols,[] )

pT.titleText = {'Expectation of Orientation dynamicstate',...
                '',...
                ''              
};
comparisonPlotFilterResultTimeSeries( tK{1}(steadyStateIdx:end), rad2deg(XUpt{1}(steadyStateIdx:end,:)),rad2deg(XUpt{2}(steadyStateIdx:end,:)),rad2deg(XUpt{3}(steadyStateIdx:end,:)), stateVar, pT,cols,[] )

pT.titleText = {'Expectation of Orientation with Compliance',...
                '',...
                ''              
};
plotFilterResultTimeSeries(tK{3}(idx:end),rad2deg(XUpt{3}(idx:end,:)),[],P(:,:,idx:end),stateVar,pT,cols,[]);



%Subtracting the state orientation with desired orientation in order to
%visualize the deviation 
XUpt{1}(idx:end,19:21) = repmat(X_des,size(tK{1},2),1) - XUpt{1}(idx:end,19:21);
XUpt{2}(idx:end,19:21) = repmat(X_des,size(tK{1},2),1) - XUpt{2}(idx:end,19:21);
XUpt{3}(idx:end,19:21) = repmat(X_des,size(tK{1},2),1) - XUpt{3}(idx:end,19:21);


pT.titleText = {'E{\phi} deviation from desired (steadystate)',...
                '',...
                ''
};
comparisonPlotFilterResultTimeSeries( tK{1}(idx:steadyStateIdx), rad2deg(XUpt{1}(idx:steadyStateIdx,:)),rad2deg(XUpt{2}(idx:steadyStateIdx,:)),rad2deg(XUpt{3}(idx:steadyStateIdx,:)), stateVar, pT,cols,[] )

pT.titleText = {'E{\phi} deviation from desired (dynamicstate)',...
                '',...
                ''
};
comparisonPlotFilterResultTimeSeries( tK{1}(steadyStateIdx:end), rad2deg(XUpt{1}(steadyStateIdx:end,:)),rad2deg(XUpt{2}(steadyStateIdx:end,:)),rad2deg(XUpt{3}(steadyStateIdx:end,:)), stateVar, pT,cols,[] )




% ax = [4 6 -10 150];
% plotFilterResultTimeSeries(tK_NC(idx:end),rad2deg(XUpt_C(idx:end,:)),rad2deg(XUpt_NC(idx:end,:)),rad2deg(XUpt_NSNC(idx:end,:)),ax,[],P(:,:,idx:end),stateVar,pT,pT2,cols,[]);
% 
% 
% 
% 
% % velocities
% stateVar = 1:6;
% pT.titleText = {'Expectation of Translation Velocity v_B',...
%                 'Expectation of Angular Velocity \omega_B',...
%                 '',...
%                 '',...
%                 '',...
%                 ''
% };
% 
% pT2.titleText = {'E{v} difference with Stiffness',...
%                 'E{\omega}difference with Stiffness',...
%                 '',...
%                 '',...
%                 '',...
%                 ''
% };
% pT.xlabelText = {'Time t(sec)',...
%                  'Time t(sec)',...
%                  'Time t(sec)',...
%                  'Time t(sec)',...
%                  'Time t(sec)',...
%                  'Time t(sec)'
% };
% pT.ylabelText = {'E(v_B_x) m/sec',...
%                  'E(\omega_B_x) rad/sec',...
%                  'E(v_B_y) N',...
%                  'E(\omega_B_y) m/sec',...
%                  'E(v_B_z) N'....
%                  'E(\omega_B_z) rad/sec'
% };
% cols = {'b','b','g','g','r','r'};
% %plotFilterResultTimeSeries(tK(idx:end),XUpt(idx:end,:),yM(idx:end,:),P(:,:,idx:end),stateVar,pT,cols,[3,2]);
% plotFilterResultTimeSeries(tK_C(idx:end),XUpt_C(idx:end,:),XUpt_NC(idx:end,:),XUpt_NSNC(idx:end,:),[],[],P(:,:,idx:end),stateVar,pT,pT2,cols,[3,2]);
% 
% 
% 
