

clear all
close all
clc

syms I_B f_B_o mu_B_o f_B_c mu_B_c m real
syms I_Bxx I_Byy I_Bzz I_Bxy I_Byz I_Bxz G_g1 G_g2 G_g3  real%G_g1 G_g2 G_g3 real
syms v_Bx v_By v_Bz real
syms f_B_ox f_B_oy f_B_oz real
syms f_B_cx f_B_cy f_B_cz real
syms omega_Bx omega_By omega_Bz real
syms mu_B_ox  mu_B_oy  mu_B_oz real
syms mu_B_cx  mu_B_cy  mu_B_cz real
syms phi1 phi2 phi3 real
syms K_B w phi0_1 phi0_2 phi0_3 real
syms K_Bxx K_Bxy K_Bxz K_Byx K_Byy K_Byz K_Bzx K_Bzy K_Bzz real

K_B = [...
     K_Bxx K_Bxy K_Bxz;...
     K_Byx K_Byy K_Byz;...
     K_Bzx K_Bzy K_Bzz];
 
wK = reshape(K_B,9,1);


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

phi0 = [phi0_1 ; phi0_2 ; phi0_3];

dv_B     = -S(omega_B) * v_B + 1/m * f_B_o - 1/m*f_B_c + B_R_G*G_g;
domega_B_withoutCompliance =  I_B \ (-S(omega_B) * (I_B * omega_B) + mu_B_o - mu_B_c);
domega_B_withCompliance =  I_B \ (-S(omega_B) * (I_B * omega_B) + mu_B_o - mu_B_c) - (K_B'*K_B)*(phi - phi0);
df_B_o   =  [0 0 0]';
dmu_B_o  =  [0 0 0]';
df_B_c   =  [0 0 0]';
dmu_B_c  =  [0 0 0]';
dphi     =  (Tomega_dphi(phi))\omega_B;


h_imu = [(dv_B - B_R_G*G_g); omega_B];
h_fto = [f_B_o; mu_B_o];
h_ftc = [f_B_c; mu_B_c];
h_skin = f_B_cz;

f_withoutCompliance     = [dv_B; domega_B_withoutCompliance; df_B_o;  dmu_B_o;df_B_c; dmu_B_c; dphi];
f_withCompliance     = [dv_B; domega_B_withCompliance; df_B_o;  dmu_B_o;df_B_c; dmu_B_c; dphi];
x     = [ v_B;  omega_B;  f_B_o;  mu_B_o; f_B_c;  mu_B_c;  phi];


df_dx_withoutCompliance = jacobian(f_withoutCompliance, x);
df_dx_withCompliance = jacobian(f_withCompliance, x);
df_dw_withCompliance = jacobian(f_withCompliance,wK);

df_dx_dualState = [df_dx_withCompliance df_dw_withCompliance;...
     zeros(9,21) eye(9)];

h_withSkin = [h_imu ; h_fto; h_ftc; h_skin];
h_withoutSkin = [h_imu ; h_fto; h_ftc];

dh_dx_withSkin = jacobian(h_withSkin,x);
dh_dx_withoutSkin = jacobian(h_withoutSkin,x);
dh_dw_withCompliance = jacobian(h_withSkin,wK);
dh_dx_dualState = [dh_dx_withSkin dh_dw_withCompliance];

model.I  = I_B;

%% WithoutCompliance, WithSkin
matlabFunction(df_dx_withoutCompliance,'file',...
    './symbolicFunctions/dynamicsDerivatives_withoutCompliance','vars',[x; dI; m; G_g]);
matlabFunction(dh_dx_withSkin,'file',....
    './symbolicFunctions/outputsDerivatives_withSkin','vars',[x; dI; m; G_g]);

%% WithoutCompliance, WithoutSkin
matlabFunction(dh_dx_withoutSkin,'file',...
    './symbolicFunctions/outputsDerivatives_withoutSkin','vars',[x; dI; m; G_g]);

%% WithCompliance, WithSkin aka dualState
matlabFunction(df_dx_dualState,'file',...
    './symbolicFunctions/dynamicsDerivatives_dualState','vars',[x; wK; dI; m; phi0; G_g]);
matlabFunction(dh_dx_dualState,'file',...
    './symbolicFunctions/outputsDerivatives_dualState','vars',[x; wK;dI; m;  phi0; G_g]);