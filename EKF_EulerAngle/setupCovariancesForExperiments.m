function [kalmanQParams,kalmanRParams,kIni] = setupCovariancesForExperiments(experiment);


%% params for experiment 1 

if(strcmp(experiment,'leg')==1)
% process model variances
% param ordering : [a_Q, omega_Q,	f_Q,	mu_Q,	phi_Q,	k_Q]
    kalmanQParams{1} = [4.0,	10.0,	5.0,	8.0,	0.5,	0.0075];  
    kalmanQParams{2} = [4.0,    10.0,   5.0,    8.0,    0.5,    0.0075]; 
    kalmanQParams{3} = [4.0,    10.0,   5.0,   8.0,     0.5,    5.0]; %0.45 
    kalmanQParams{4} = [4.0,    10.0,   5.0,    8.0,    0.5,    0.0075]; 
    kalmanQParams{5} = [4.0,    10.0,   5.0,    8.0,    0.5,    0.0075]; 
% measurement model variances
% param ordering : [f_R,    mu_R,   a_R,    omega_R,    skin_R]
    kalmanRParams{1} = [1.5,    2.75,   1.25,   4.5,        25.75];    
    kalmanRParams{2} = [1.5,    2.75,   1.25,   4.5,        25.75];
    kalmanRParams{3} = [1.5,    2.75,   1.25,   4.5,        25.75];
    kalmanRParams{4} = [1.5,    2.75,   1.25,   4.5,        25.75];
    kalmanRParams{5} = [1.5,    2.75,   1.25,   4.5,        25.75];
end


if(strcmp(experiment,'foot')==1)
% process model variances
% param ordering : [a_Q, omega_Q,	f_Q,	mu_Q,	phi_Q,	k_Q]
    kalmanQParams{1} = [4.0,    10.0,   5.0,   8.0,     0.5,    0.0075]; %0.45 
    kalmanQParams{2} = [4.0,    10.0,   5.0,    8.0,    0.5,    0.0075]; 
    
% measurement model variances
% param ordering : [f_R,    mu_R,   a_R,    omega_R,    skin_R]
    kalmanRParams{1} = [1.5,    2.75,   1.25,   4.5,        25.75];    
    kalmanRParams{2} = [1.5,    2.75,   1.25,   4.5,        25.75];
end


% initial stiffness (defined but not used for all experiments)
% kIni = 2.975;
kIni = 1.68339014; %btween 1.6833901 and 1.6833902
end