clear all
close all
clc


% syms f mu m gz real
% syms P_ank P_com P_SI real
% syms P_com_x P_com_y P_com_z real
% syms P_ank_x P_ank_y P_ank_z real
% syms f_x f_y f_z real
% syms mu_x  mu_y  mu_z real
% syms z A_SI B_SI

%syms f_bar mu_bar P_ank_bar P_com_bar m gz real
%syms f_mean mu_mean P_ank_mean P_com_mean real
%syms f_var mu_var P_ank_var P_com_var real
syms m gz real

f_bar = sym('f_bar',[3,1]);
mu_bar = sym('mu_bar',[3,1]);
P_ank_bar = sym('P_ank_bar',[3,1]);
P_com_bar = sym('P_com_bar',[3,1]);

f_mean = sym('f_mean',[3,1]);
mu_mean = sym('mu_mean',[3,1]);
P_ank_mean = sym('P_ank_mean',[3,1]);
P_com_mean = sym('P_com_mean',[3,1]);

f_var = sym('f_var',[3,1]);
mu_var = sym('mu_var',[3,1]);
P_ank_var = sym('P_ank_var',[3,1]);
P_com_var = sym('P_com_var',[3,1]);

%syms z_bar z_mean z_var real
% z_bar = sym('z_bar',[12 1]);
% z_mean = sym('z_mean',[12 1]);
% z_var = sym('z_var',[12 1]);
z = sym('z',[12 1]);
f = sym('f',[3 1]);
mu = sym('mu',[3,1]);
P_ank = sym('P_ank',[3,1]);
P_com = sym('P_com',[3,1]);
%syms Fx_full(f,mu,P_ank,P_com) Fy_full(f,mu,P_ank,P_com) real
%syms Fx_z(z) Fy_z(z) F(z) real
%syms F_bar(z) F_mean(z)% J_bar J_mean
syms p_expect_atBar p_cov_atBar p_expect_atMean p_cov_atMean real

% 
% f = [f_x f_y f_z]';
% mu = [mu_x mu_y mu_z]';
% P_ank = [P_ank_x P_ank_y P_ank_z]';
% P_com = [P_com_x P_com_y P_com_z]';

z_bar = [f_bar;mu_bar;P_ank_bar;P_com_bar];
z_mean = [f_mean;mu_mean;P_ank_mean;P_com_mean];
z_var = [f_var;mu_var;P_ank_var;P_com_var];

%Fx = @(f,mu,P_ank,P_com)((-mu_y - P_ank_z*f_x + P_ank_x*f_z + m*gz * P_com_x)./(m*gz + f_z));
%Fy = @(f,mu,P_ank,P_com)((-mu_x - P_ank_z*f_y + P_ank_y*f_z + m*gz * P_com_y)./(m*gz + f_z));

Fx_full = @(f,mu,P_ank,P_com)((-mu(2) - P_ank(3)*f(1) + P_ank(1)*f(3) + m*gz * P_com(1))./(m*gz + f(3)));
Fy_full = @(f,mu,P_ank,P_com)((-mu(1) - P_ank(3)*f(2) + P_ank(2)*f(3) + m*gz * P_com(2))./(m*gz + f(3)));

Fx_z = @(z)Fx_full(z(1:3),z(4:6),z(7:9),z(10:12));
Fy_z = @(z)Fy_full(z(1:3),z(4:6),z(7:9),z(10:12));


F = @(z)[Fx_z(z);Fy_z(z)];

F_bar = F(z_bar);
F_mean = F(z_mean);
%F_var 
%Jx = jacobian(Fx,z);

%Jx1 = jacobian(Fx,f);
%Jx2 = jacobian(Fx,mu);
%Jx3 = jacobian(Fx,P_ank);
%Jx4 = jacobian(Fx,P_com);

% Jy1 = jacobian(Fy,f);
% Jy2 = jacobian(Fy,mu);
% Jy3 = jacobian(Fy,P_ank);
% Jy4 = jacobian(Fy,P_com);
% 
% Jx = [Jx1 Jx2 Jx3 Jx4];
% Jy = [Jy1 Jy2 Jy3 Jy4];

J_bar = jacobian(F_bar,z_bar);
J_mean = jacobian(F_mean,z_mean);

%Jy = jacobian(Fz,z);

%A_SI = [Jx;Jy];
%B_SI = [Jx;Jy]*z;

p_expect_at_Bar = J_bar*z_mean + J_bar*z_bar + F_bar;
p_cov_at_Bar = J_bar*diag(z_var)*J_bar';
%[Jx_mean;Jy_mean

p_expect_at_Mean = F_mean;
p_cov_at_Mean = J_mean*diag(z_var)*J_mean';

matlabFunction(p_expect_at_Bar,p_cov_at_Bar,'file','stabilityIndex_arbitrayLinearisation','vars',[[f_bar; mu_bar; P_ank_bar; P_com_bar];[f_mean; mu_mean; P_ank_mean; P_com_mean];...
    [f_var; mu_var; P_ank_var; P_com_var]; m ; gz],'outputs',{'p_expect','p_covariance'});
matlabFunction(p_expect_at_Mean,p_cov_at_Mean,'file','stabilityIndex_meanLinearisation','vars',[[f_mean; mu_mean; P_ank_mean; P_com_mean];[f_var; mu_var; P_ank_var; P_com_var]; m ; gz]...
    ,'outputs',{'p_expect','p_covariance'});


matlabFunction(p_expect_at_Bar,p_cov_at_Bar,'file','stabilityIndex_arbitrayLinearisation_zin','vars',[z_bar;z_mean;...
    z_var; m ; gz],'outputs',{'p_expect','p_covariance'});
matlabFunction(p_expect_at_Mean,p_cov_at_Mean,'file','stabilityIndex_meanLinearisation_zin','vars',[z_mean;z_var; m ; gz]...
    ,'outputs',{'p_expect','p_covariance'});