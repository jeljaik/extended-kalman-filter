function [kalmanQParams,kalmanRParams,k,c] = setupCovariancesForExperiments_foot(filter);

%kalmanQParams = cell(3);
%% params for experiment 1 
% process model variances
% param ordering : [a_Q, omega_Q,	f_Q,	mu_Q,	phi_Q,	k_Q, c_Q]
% kalmanQParams = [4.0,	10.0,	5.0,	8.0,	0.5,	0.0075];  

% kalmanQParams = [0.025,	0.0125,	0.25,	0.025,	0.125, 0.001595, 0.0000125];  % acceptable params ----> EKF

% kalmanQParams = [0.05,	0.0125,	0.05,	0.025,	0.0, 0.001595, 0.00000125];  % acceptable params ----> JEKF, EKF

if(strcmp(filter,'jekf') == 1 || strcmp(filter,'ekf') == 1 )
% kalmanQParams = [0.005,	0.0125,	0.05,	0.025,	0.001, 0.001595, 0.00000125];  % good params ----> JEKF, EKF
% kalmanQParams = [0.005,	0.0005,	0.05,	0.05,	0.00, 0.001595, 0.00000125];
kalmanQParams = [4.0,	0.5,	2.0,	2.0,	0.0025,	0.2595, 0.00025];
else if(strcmp(filter,'dekf') == 1)
% kalmanQParams = [0.005,	0.0005,	0.05,	0.05,	0.00, 5, 0.25];  % acceptable params ----> DEKF,JEKF
    kalmanQParams = [4.0,	0.5,	2.0,	2.0,	0.0025,	0.25, 0.00025];  
    end
end
%  kalmanQParams = [0.25,	0.25,	0.25,	0.25,	0.25, 0.25, 0.25];  


% measurement model variances
% param ordering : [f_R,    mu_R,   a_R,    omega_R,    skin_R]
kalmanRParams = [1.5,    2.75,   1.25,   4.5,        25.75];    

% kalmanRParams = [0.5,    0.5,  1.5,   0.5,   0.5];    % acceptable params -------> EKF


% kalmanRParams = [0.5,    0.5,  0.25,   0.5,   0.5];    % acceptable params -------> EKF

% kalmanRParams = [0.001,    0.001,  0.0001,   0.0001,   0.5];    % actual params in principle
% kalmanRParams = [0.5,    0.25,  0.25,   0.75,   0.5];   % acceptable params ---->DEKF


% initial stiffness (defined but not used for all experiments)
%k = [k_xx; k_yy; k_zz]
k = [0.0; 0.0; 0.0];

% initial damping (defined but not used for all experiments)
%c = [c_xx; c_yy; c_zz]
c = [0.0; 0.0; 0.0];


end
