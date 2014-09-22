clear all
close all
clc


syms f mu m gz real
syms P_ank P_com P_SI real
syms P_com_x P_com_y P_com_z real
syms P_ank_x P_ank_y P_ank_z real
syms f_x f_y f_z real
syms mu_x  mu_y  mu_z real
syms z A_SI B_SI


f = [f_x f_y f_z]';
mu = [mu_x mu_y mu_z]';
P_ank = [P_ank_x P_ank_y P_ank_z]';
P_com = [P_com_x P_com_y P_com_z]';

z = [f;mu;P_ank;P_com];

Fx = (-mu_y - P_ank_z*f_x + P_ank_x*f_z + m*gz * P_com_x)./(m*gz + f_z);
Fy = (-mu_x - P_ank_z*f_y + P_ank_y*f_z + m*gz * P_com_y)./(m*gz + f_z);

%Jx = jacobian(Fx,z);
Jx1 = jacobian(Fx,f);
Jx2 = jacobian(Fx,mu);
Jx3 = jacobian(Fx,P_ank);
Jx4 = jacobian(Fx,P_com);

Jy1 = jacobian(Fy,f);
Jy2 = jacobian(Fy,mu);
Jy3 = jacobian(Fy,P_ank);
Jy4 = jacobian(Fy,P_com);

Jx = [Jx1 Jx2 Jx3 Jx4];
Jy = [Jy1 Jy2 Jy3 Jy4];
%Jy = jacobian(Fz,z);

A_SI = [Jx;Jy];
B_SI = [Jx;Jy]*z;

matlabFunction(A_SI,B_SI,'file','linearisedStabilityIndex','vars',[[f; mu; P_ank; P_com]; m ; gz],'outputs',{'A','B'});