clear 
close all
clc

%% Utils folder contains the function euler2dcm()
utils =  genpath('./../utils');
addpath(utils);

%% Initialise the symbolic variables
syms I_B f_B_o mu_B_o f_B_c mu_B_c m real
syms I_Bxx I_Byy I_Bzz I_Bxy I_Byz I_Bxz G_g1 G_g2 G_g3  real
syms v_Bx v_By v_Bz real
syms f_B_ox f_B_oy f_B_oz real
syms f_B_cx f_B_cy f_B_cz real
syms omega_Bx omega_By omega_Bz real
syms mu_B_ox  mu_B_oy  mu_B_oz real
syms mu_B_cx  mu_B_cy  mu_B_cz real
syms phi1 phi2 phi3 real
syms K C w real
syms k_xx k_yy k_zz c_xx c_yy c_zz real
syms phi01 phi02 phi03 real % rest position

K = diag([k_xx,k_yy,k_zz]);
C = diag([c_xx,c_yy,c_zz]);
w = [k_xx k_yy k_zz c_xx c_yy c_zz]';

I_B = [...
     I_Bxx I_Bxy I_Bxz;...
     I_Bxy I_Byy I_Byz;...
     I_Bxz I_Byz I_Bzz];
dI = [I_Bxx I_Bxy I_Bxz I_Byy I_Byz I_Bzz ]';

omega_B = [    omega_Bx   omega_By   omega_Bz]';
v_B     = [    v_Bx       v_By       v_Bz]';
f_B_o   = [    f_B_ox     f_B_oy     f_B_oz]';
f_B_c   = [    f_B_cx     f_B_cy     f_B_cz]';
mu_B_o  = [    mu_B_ox    mu_B_oy    mu_B_oz]';
mu_B_c  = [    mu_B_cx    mu_B_cy    mu_B_cz]';
phi     = [    phi1       phi2       phi3]';
B_R_G       = euler2dcm(phi);
G_g = [G_g1;G_g2;G_g3];

phi0    = [    phi01      phi02      phi03]'; %rest orientation used by the spring torque

dv_B     = -S(omega_B) * v_B + 1/m * f_B_o - 1/m*f_B_c + euler2dcm(phi)*G_g;
domega_B_withCompliance =  I_B \ (-S(omega_B) * (I_B * omega_B) + mu_B_o - mu_B_c  + (- K'*K*(phi - phi0) - C*omega_B));
domega_B_withoutCompliance =  I_B \ (-S(omega_B) * (I_B * omega_B) + mu_B_o - mu_B_c);
df_B_o   =  [0 0 0]';
dmu_B_o  =  [0 0 0]';
df_B_c   =  [0 0 0]';
dmu_B_c  =  [0 0 0]';
dphi     =  (Tomega_dphi(phi))\omega_B;


h_imu = [(dv_B - B_R_G*G_g); omega_B];
h_fto = [f_B_o; mu_B_o];
% h_ftc = [f_B_c; mu_B_c];
h_skin = f_B_cz;

x     = [ v_B;  omega_B;  f_B_o;  mu_B_o; f_B_c;  mu_B_c;  phi];
dw    = zeros(6,1);

%% Process model

f_withCompliance     = [dv_B; domega_B_withCompliance; df_B_o;  dmu_B_o;df_B_c; dmu_B_c; dphi]; %EKF,DEKF
f_withoutCompliance     = [dv_B; domega_B_withoutCompliance; df_B_o;  dmu_B_o;df_B_c; dmu_B_c; dphi]; %EKF
f_jointState = [f_withCompliance;dw]; %JEKF

df_dx_withoutCompliance = jacobian(f_withoutCompliance, x); %EKF

df_dx_withCompliance = jacobian(f_withCompliance, x); %EKF,DEKF
df_dw = jacobian(f_withCompliance,w); %JEKF
df_dx_jointState = [df_dx_withCompliance df_dw;...
      zeros(size(df_dw')) eye(size(df_dw,2))];      %JEKF

dp_dw = jacobian(dw,w);  %DEKF  

%% Measurement model
h_withSkin = [h_imu ; h_fto; h_skin];
h_withoutSkin = [h_imu ; h_fto];

dh_dx_withSkin = jacobian(h_withSkin,x); %EKF,DEKF
dh_dx_withoutSkin = jacobian(h_withoutSkin,x);  %EKF,DEKF

dh_dw_withSkin = jacobian(h_withSkin,w); %JEKF
dh_dw_withoutSkin = jacobian(h_withoutSkin,w); %JEKF

dh_dx_jointStateWithSkin = [dh_dx_withSkin dh_dw_withSkin];  %JEKF
dh_dx_jointStateWithoutSkin = [dh_dx_withoutSkin dh_dw_withoutSkin];  %JEKF


dg_dw_withSkin = dh_dx_withSkin*jacobian(f_withCompliance,w); %DEKF
dg_dw_withoutSkin = dh_dx_withoutSkin*jacobian(f_withCompliance,w); %DEKF
model.I  = I_B;

%% Regular EKF

matlabFunction(f_withCompliance,'file','./../dynamicsFunctions/processODE_withCompliance','vars',[x; w; phi0; dI; m; G_g]);
matlabFunction(df_dx_withCompliance,'file','./dynamicsDerivatives_withCompliance','vars',[x; w; phi0; dI; m; G_g]);

matlabFunction(f_withoutCompliance,'file','./../dynamicsFunctions/processODE_withoutCompliance','vars',[x; w; phi0; dI; m; G_g]);
matlabFunction(df_dx_withoutCompliance,'file','./dynamicsDerivatives_withoutCompliance','vars',[x; w; phi0; dI; m; G_g]);

matlabFunction(h_withoutSkin,'file','./../dynamicsFunctions/measurement_withoutSkin','vars',[x; w; phi0; dI; m; G_g]);
matlabFunction(dh_dx_withoutSkin,'file','./outputsDerivatives_withoutSkin','vars',[x; w; phi0; dI; m; G_g]);

matlabFunction(h_withSkin,'file','./../dynamicsFunctions/measurement_withSkin','vars',[x; w; phi0; dI; m; G_g]);
matlabFunction(dh_dx_withSkin,'file','./outputsDerivatives_withSkin','vars',[x; w; phi0; dI; m; G_g]);

%% JEKF

matlabFunction(f_jointState,'file','./../dynamicsFunctions/processODE_jointState','vars',[x; w; phi0; dI; m; G_g]);
matlabFunction(df_dx_jointState,'file','./dynamicsDerivatives_jointState','vars',[x; w; phi0; dI; m; G_g]);
matlabFunction(dh_dx_jointStateWithoutSkin,'file','./outputsDerivatives_jointStateWithoutSkin','vars',[x; w; phi0; dI; m; G_g]);
matlabFunction(dh_dx_jointStateWithSkin,'file','./outputsDerivatives_jointStateWithSkin','vars',[x; w; phi0; dI; m; G_g]);

%% DEKF
matlabFunction(dw,'file','./../dynamicsFunctions/paramODE_dualState','vars',[x; w; phi0; dI; m; G_g]);
matlabFunction(dp_dw,'file','./paramdynamicsDerivatives_dualState','vars',[x; w; phi0; dI; m; G_g]);
matlabFunction(dg_dw_withSkin,'file','./paramoutputsDerivatives_dualStateWithSkin','vars',[x; w; phi0; dI; m; G_g]);
matlabFunction(dg_dw_withoutSkin,'file','./paramoutputsDerivatives_dualStateWithoutSkin','vars',[x; w; phi0; dI; m; G_g]);
