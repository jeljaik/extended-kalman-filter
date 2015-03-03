clear all
close all
clc


syms fc_z_bar  muc_x_bar muc_y_bar muo_x_bar fo_z_bar muo_y_bar fo_z_bar real
syms fc_z_mean  muc_x_mean muc_y_mean muo_x_mean fo_z_mean muo_y_mean fo_z_mean real
syms fc_z_var  muc_x_var muc_y_var muo_x_var fo_z_var muo_y_var fo_z_var real
syms F_cop(muc_x,muc_y,fc_z) F_fri(muo_x, muo_y, fo_z) real
syms F_cop_bar F_cop_mean F_fri_bar F_fri_mean real
syms J_cop_bar J_cop_mean J_fri_bar J_fri_mean real
syms P_c_mean P_o_mean

syms pfri_expect_atBar pfri_cov_atBar pfri_expect_atMean pfri_cov_atMean real

%F_cop = @(muc_x,muc_y,fc_z)[ -muc_x/fc_z ; muc_y/fc_z ];
%F_fri = @(muo_x,muo_y,fo_z)[  muo_x/fo_z ; muo_y/fo_z ];

%F_cop_bar  = @(muc_x_bar,muc_y_bar,fc_z_bar)[ -muc_x_bar/fc_z_bar ; muc_y_bar/fc_z_bar ];
%F_cop(muc_x_bar, muc_y_bar,fc_z_bar);
F_cop_mean = [ -muc_x_mean/fc_z_mean ; muc_y_mean/fc_z_mean ];
%F_cop(muc_x_mean, muc_y_mean,fc_z_mean);

%F_fri_bar = F_fri(muo_x_bar, muo_y_bar,fo_z_bar);
F_fri_mean = [muo_x_mean/fo_z_mean ; muo_y_mean/fo_z_mean ];
%F_fri = @(muo_x,muo_y,fo_z)[  muo_x/fo_z ; muo_y/fo_z ];

P_c_mean = [muc_x_mean, muc_y_mean,fc_z_mean]';
P_o_mean = [muo_x_mean, muo_y_mean,fo_z_mean]';

%J_cop_bar = jacobian(F_cop_bar,[muc_x_bar, muc_y_bar,fc_z_bar]');
J_cop_mean = jacobian(F_cop_mean,P_c_mean);

%J_fri_bar = jacobian(F_fri_bar,[muo_x_bar, muo_y_bar,fo_z_bar]');
J_fri_mean = jacobian(F_fri_mean,P_o_mean);


%pcop_expect_at_Bar = J_cop_bar*[muc_x_mean, muc_y_mean,fc_z_mean]' + J_cop_bar*[muc_x_bar, muc_y_bar,fc_z_bar]' + F_cop_bar;
%pcop_cov_at_Bar = J_cop_bar*diag([muc_x_var, muc_y_var,fc_z_var]')*J_cop_bar';
pcop_expect_at_Mean = F_cop_mean;
pcop_cov_at_Mean = J_cop_mean*diag([muc_x_var, muc_y_var,fc_z_var]')*J_cop_mean';

%pfri_expect_at_Bar = J_fri_bar*[muo_x_mean, muo_y_mean,fo_z_mean]' + J_fri_bar*[muo_x_bar, muo_y_bar,fo_z_bar]' + F_fri_bar;
%pfri_cov_at_Bar = J_fri_bar*diag([muo_x_var, muo_y_var,fo_z_var]')*J_fri_bar';
pfri_expect_at_Mean = F_fri_mean;
pfri_cov_at_Mean = J_fri_mean*diag([muo_x_var, muo_y_var,fo_z_var]')*J_fri_mean';
% 
% 
% matlabFunction(pfri_expect_at_Bar,pfri_cov_at_Bar,'file','fri_arbitrayLinearisation','vars',[[muc_x_bar, muc_y_bar,fc_z_bar]';[muc_x_mean, muc_y_mean,fc_z_mean]';...
%     [muo_x_var, muo_y_var,fo_z_var]'],'outputs',{'pfri_expect','p_covariance'});
matlabFunction(pfri_expect_at_Mean,pfri_cov_at_Mean,'file','./symbolic/computeFRI','vars',[[muo_x_mean, muo_y_mean,fo_z_mean],[muo_x_var, muo_y_var,fo_z_var]],'outputs',{'pfri_expect','pfri_covariance'});

matlabFunction(pcop_expect_at_Mean,pcop_cov_at_Mean,'file','./symbolic/computeCOP','vars',[[muc_x_mean, muc_y_mean,fc_z_mean],[muc_x_var, muc_y_var,fc_z_var]],'outputs',{'pcop_expect','pcop_covariance'});

%matlabFunction(pfri_expect_at_Mean,pfri_cov_at_Mean,'file','./symbolic/fri_meanLinearisation','vars',[[muc_x_mean, muc_y_mean,fc_z_mean]';...
    %[muo_x_var, muo_y_var,fo_z_var]'],'outputs',{'p_expect','p_covariance'});

% 
% matlabFunction(p_expect_at_Bar,p_cov_at_Bar,'file','stabilityIndex_arbitrayLinearisation_zin','vars',[z_bar;z_mean;...
%     z_var; m ; gz],'outputs',{'p_expect','p_covariance'});
% matlabFunction(p_expect_at_Mean,p_cov_at_Mean,'file','stabilityIndex_meanLinearisation_zin','vars',[z_mean;z_var; m ; gz]...
%     ,'outputs',{'p_expect','p_covariance'});