function [kalmanQParams,kalmanRParams,kIni] = setupCovariancesForExperiments()

kalmanQParams = cell(3);
%% params for experiment 1 
% process model variances
% param ordering : [a_Q, omega_Q,	f_Q,	mu_Q,	phi_Q,	k_Q]
kalmanQParams{1} = [4.0,	10.0,	5.0,	8.0,	0.5,	0.0075];  
kalmanQParams{2} = 0.1*[4.0,    10.0,   5.0,    8.0,    0.05,    0.0075]; 
kalmanQParams{3} = 0.1*[4.0,    2.50,   5.0,    8.0,    0.015,    0.75]; 

% measurement model variances
% param ordering : [f_R,    mu_R,   a_R,    omega_R,    skin_R]
kalmanRParams{1} = [1.5,    2.75,   1.25,   4.5,        2.575];    
kalmanRParams{2} = 2.5*[1.5,    2.75,   0.55,   0.5,        2.575];
kalmanRParams{3} = 0.5*[1.5,    2.75,   0.55,   0.5,        2.575];

% initial stiffness (defined but not used for all experiments)
kIni = 0;%00.175;
end