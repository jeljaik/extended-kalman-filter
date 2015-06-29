clc;
close all;
clear all;
% Loading compliance results
dataBaseFolder = cell();

numOfExperiments = 1:3; %number of experiments
% 1 -  no stiffness and no skin
% 2 -  no stiffness and including skin
% 3 - including stiffness and including skin

for expID = 1:3
	dataBaseFolder= sprintf('./data/humanoids2015/Exp_%d/',expID);
    load(strcat(dataBaseFolder,'filteredResult.mat'));

tK_C = tKalman;
yM_C = yMeas;
source_C = 2;
XUpt_C = Xupdt;
XPred_C = Xhat;
tMin_C = tK_C(1);


% Loading no compliance results
dataBaseFolder2 = 'NoCompliance/data/';
load(strcat(dataBaseFolder2,'filter_result_data.mat'));
tK_NC = tKalman;
yM_NC = yMeas;
source_NC = 2;
XUpt_NC = Xupdt;
XPred_NC = Xhat;
tMin_NC = tK_NC(1);




% Loading no skin no compliance results
dataBaseFolder3 = 'NoSkinNoCompliance/data/';
load(strcat(dataBaseFolder3,'filter_result_data.mat'));
tK_NSNC = tKalman;
yM_NSNC = yMeas;
source_NSNC = 2;
XUpt_NSNC = Xupdt;
XPred_NSNC = Xhat;
tMin_NSNC = tK_NSNC(1);



idx = 1;

% orientation
stateVar = 19:21;
pT.titleText = {'Expectation of Orientation',...
                '',...
                ''              
};

pT2.titleText = {'E{\phi} difference with stiffness',...
                '',...
                ''
};

pT.xlabelText = {'Time t(sec)',...
                 'Time t(sec)',...
                 'Time t(sec)'
};
pT.ylabelText = {'E(\phi_x) degs',...
                 'E(\phi_y) degs',...
                 'E(\phi_z) degs'
};
cols = {'b','g','r'};
ax = [4 6 -10 150];
plotFilterResultTimeSeries(tK_NC(idx:end),rad2deg(XUpt_C(idx:end,:)),rad2deg(XUpt_NC(idx:end,:)),rad2deg(XUpt_NSNC(idx:end,:)),ax,[],P(:,:,idx:end),stateVar,pT,pT2,cols,[]);




% velocities
stateVar = 1:6;
pT.titleText = {'Expectation of Translation Velocity v_B',...
                'Expectation of Angular Velocity \omega_B',...
                '',...
                '',...
                '',...
                ''
};

pT2.titleText = {'E{v} difference with Stiffness',...
                'E{\omega}difference with Stiffness',...
                '',...
                '',...
                '',...
                ''
};
pT.xlabelText = {'Time t(sec)',...
                 'Time t(sec)',...
                 'Time t(sec)',...
                 'Time t(sec)',...
                 'Time t(sec)',...
                 'Time t(sec)'
};
pT.ylabelText = {'E(v_B_x) m/sec',...
                 'E(\omega_B_x) rad/sec',...
                 'E(v_B_y) N',...
                 'E(\omega_B_y) m/sec',...
                 'E(v_B_z) N'....
                 'E(\omega_B_z) rad/sec'
};
cols = {'b','b','g','g','r','r'};
%plotFilterResultTimeSeries(tK(idx:end),XUpt(idx:end,:),yM(idx:end,:),P(:,:,idx:end),stateVar,pT,cols,[3,2]);
plotFilterResultTimeSeries(tK_C(idx:end),XUpt_C(idx:end,:),XUpt_NC(idx:end,:),XUpt_NSNC(idx:end,:),[],[],P(:,:,idx:end),stateVar,pT,pT2,cols,[3,2]);



stateVar = 22:30;
for j = idx : size(XUpt_C,1)
    K_sqr(j) = 0;
    for i = 1 : size(stateVar,2)
         
          K_sqr(j)=K_sqr(j) + XUpt_C(j,stateVar(i)).^2;
          
     
    end    
     K_norm(j) = K_sqr(j).^0.5;
end

figure();
plot(tK_C(idx:end),K_norm(idx:end,:));
title('Stiffness norm evolution');
xlabel('Time (s)');
ylabel('|K| Nm/rad');