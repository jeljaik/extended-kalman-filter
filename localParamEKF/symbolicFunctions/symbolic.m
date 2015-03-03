 clear all
close all
clc

syms I_B f_B_o mu_B_o f_B_c mu_B_c m real
syms I_Bxx I_Byy I_Bzz I_Bxy I_Byz I_Bxz g real
syms v_Bx v_By v_Bz real
syms f_B_ox f_B_oy f_B_oz real
syms f_B_cx f_B_cy f_B_cz real
syms omega_Bx omega_By omega_Bz real
syms mu_B_ox  mu_B_oy  mu_B_oz real
syms mu_B_cx  mu_B_cy  mu_B_cz real
syms phi1 phi2 phi3 real


I_B = [...
     I_Bxx I_Bxy I_Bxz;...
     I_Bxy I_Byy I_Byz;...
     I_Bxz I_Byz I_Bzz];
dI = [I_Bxx I_Bxy I_Bxz I_Byy I_Byz I_Bzz ]';
 
%dI = [I_Bxx I_Byy I_Bzz]';
%I_B = diag(dI);

omega_B = [    omega_Bx   omega_By   omega_Bz]';
v_B     = [    v_Bx       v_By       v_Bz]';
f_B_o   = [    f_B_ox     f_B_oy     f_B_oz]';
f_B_c   = [    f_B_cx     f_B_cy     f_B_cz]';
mu_B_o  = [    mu_B_ox    mu_B_oy    mu_B_oz]';
mu_B_c  = [    mu_B_cx    mu_B_cy    mu_B_cz]';
phi     = [    phi1       phi2       phi3]';
R       = euler2dcm(phi);

dv_B     = -S(omega_B) * v_B + 1/m * f_B_o - 1/m*f_B_c + g.*R*[1; 0; 0];
domega_B =  I_B \ (-S(omega_B) * (I_B * omega_B) + mu_B_o - mu_B_c);
df_B_o   =  [0 0 0]';
dmu_B_o  =  [0 0 0]';
df_B_c   =  [0 0 0]';
dmu_B_c  =  [0 0 0]';
% 
% Tphi = [ 0 -sin(phi1) cos(phi1)*sin(phi2) ;...
%         0 cos(phi1)    sin(phi1)*sin(phi2) ;...
%         1 0 cos(phi2)];
%dphi = Tphi\omega_B;
%dphi = Tomega_dphi(phi
%R\omega_B;
dphi     =  (Tomega_dphi(phi))\omega_B;


h_imu = [dv_B + g.*R*[1; 0; 0]; omega_B];
%h_imu = [dv_B ; omega_B];
h_fto = [f_B_o; mu_B_o];
h_ftc = [f_B_c; mu_B_c];
h_skin = f_B_cz;
%h_skin = [f_B_c_x; mu_B_cy; mu_B_cz];

f     = [dv_B; domega_B; df_B_o;  dmu_B_o;df_B_c; dmu_B_c; dphi];
x     = [ v_B;  omega_B;  f_B_o;  mu_B_o; f_B_c;  mu_B_c;  phi];
%h_noSkin_noGyro = [dv_B; f_B_1; f_B_2; mu_B_1; mu_B_2];
%h = [dv_B; omega_B; f_B_1; mu_B_1; f_B_2x; mu_B_2y;mu_B_2z];
%h_noSkin_withGyro = [dv_B; omega_B; f_B_1; f_B_2; mu_B_1; mu_B_2];

h = [h_imu ; h_fto; h_ftc; h_skin];

df_dx = jacobian(f, x);
%dh_dx_noSkin_noGyro = jacobian(h_noSkin_withGyro, x);
%dh_dx_withSkin_withGyro = jacobian(h_withSkin_withGyro,x);
dh_dx = jacobian(h,x);

model.I  = I_B;

%matlabFunction(df_dx,'file','./symbolic/rigidBodyDynamicsDerivatives','vars',[x; dI; m; g]);

%matlabFunction(dh_dx,'file','./symbolic/rigidBodyOutputsDerivatives','vars',[x; dI; m; g]);
matlabFunction(df_dx,'file','./symbolicFunctions/rigidBodyDynamicsDerivatives','vars',[x; dI; m; g]);

matlabFunction(dh_dx,'file','./symbolicFunctions/rigidBodyOutputsDerivatives','vars',[x; dI; m; g]);
