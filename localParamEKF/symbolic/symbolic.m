 clear all
close all
clc

syms I_B f_B_1 f_B_2 mu_B_1 mu_B_2 m real
syms I_Bxx I_Byy I_Bzz I_Bxy I_Byz I_Bxz g real
syms v_Bx v_By v_Bz real
syms f_B_1x f_B_1y f_B_1z real
syms f_B_2x f_B_2y f_B_2z real
syms omega_Bx omega_By omega_Bz real
syms mu_B_1x  mu_B_1y  mu_B_1z real
syms mu_B_2x  mu_B_2y  mu_B_2z real
syms phi1 phi2 phi3 real


% I_B = [...
%     I_Bxx I_Bxy I_Bxz
%     I_Bxy I_Byy I_Byz;
%     I_Bxz I_Byz I_Bzz];

dI = [I_Bxx I_Byy I_Bzz]';
I_B = diag(dI);

omega_B = [    omega_Bx   omega_By   omega_Bz]';
v_B     = [    v_Bx       v_By       v_Bz]';
f_B_1   = [    f_B_1x     f_B_1y     f_B_1z]';
f_B_2   = [    f_B_2x     f_B_2y     f_B_2z]';
mu_B_1  = [    mu_B_1x    mu_B_1y    mu_B_1z]';
mu_B_2  = [    mu_B_2x    mu_B_2y    mu_B_2z]';
phi     = [    phi1       phi2       phi3]';
R       = euler2dcm(phi);

dv_B     = -S(omega_B) * v_B + 1/m * f_B_1 + 1/m*f_B_2 + g.*R*[0; 0; 1];
domega_B =  I_B \ (-S(omega_B) * (I_B * omega_B) + mu_B_1 + mu_B_2);
df_B_1   =  [0 0 0]';
df_B_2   =  [0 0 0]';
dmu_B_1  =  [0 0 0]';
dmu_B_2  =  [0 0 0]';
dphi     =  inv(Tomega_dphi(phi))*omega_B;

f     = [dv_B; domega_B; df_B_1; df_B_2; dmu_B_1; dmu_B_2; dphi];
x     = [ v_B;  omega_B;  f_B_1; f_B_2;  mu_B_1; mu_B_2;  phi];
%h_noSkin_noGyro = [dv_B; f_B_1; f_B_2; mu_B_1; mu_B_2];
h = [dv_B; omega_B; f_B_1; mu_B_1; f_B_2x; mu_B_2y;mu_B_2z];
%h_noSkin_withGyro = [dv_B; omega_B; f_B_1; f_B_2; mu_B_1; mu_B_2];

df_dx = jacobian(f, x);
%dh_dx_noSkin_noGyro = jacobian(h_noSkin_withGyro, x);
%dh_dx_withSkin_withGyro = jacobian(h_withSkin_withGyro,x);
dh_dx = jacobian(h,x);

model.I  = I_B;

matlabFunction(df_dx,'file','./symbolic/rigidBodyDynamicsDerivatives','vars',[x; dI; m; g]);
%matlabFunction(dh_dx_noSkin_noGyro,'file','./symbolic/rigidBodyOutputsDerivatives_noSkin_noGyro','vars',[x; dI; m; g]);

%matlabFunction(df_dx,'file','./symbolic/rigidBodyDynamicsDerivatives_noGyro','vars',[x; dI; m; g]);
matlabFunction(dh_dx,'file','./symbolic/rigidBodyOutputsDerivatives','vars',[x; dI; m; g]);
%matlabFunction(dh_dx_noSkin_withGyro,'file','./symbolic/rigidBodyOutputsDerivatives','vars',[x; dI; m; g]);
