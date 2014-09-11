clear all
close all
clc

syms I_B f_B mu_B m real
syms I_Bxx I_Byy I_Bzz I_Bxy I_Byz I_Bxz g real
syms v_Bx v_By v_Bz real
syms f_Bx f_By f_Bz real
syms omega_Bx omega_By omega_Bz real
syms    mu_Bx    mu_By    mu_Bz real
syms phi1 phi2 phi3 real


% I_B = [...
%     I_Bxx I_Bxy I_Bxz
%     I_Bxy I_Byy I_Byz;
%     I_Bxz I_Byz I_Bzz];

dI = [I_Bxx I_Byy I_Bzz]';
I_B = diag(dI);

omega_B = [omega_Bx omega_By omega_Bz]';
v_B     = [    v_Bx     v_By     v_Bz]';
f_B     = [    f_Bx     f_By     f_Bz]';
mu_B    = [   mu_Bx    mu_By    mu_Bz]';
phi     = [    phi1     phi2     phi3]';
R       = euler2dcm(phi);

dv_B     =          -S(omega_B) * v_B + 1/m * f_B + g.*R*[0; 0; 1];
domega_B =   I_B \ (-S(omega_B) * (I_B * omega_B) + mu_B);
df_B     =                                [0 0 0]';
dmu_B    =                                [0 0 0]';
dphi     =                inv(Tomega_dphi(phi))*omega_B;

f  = [dv_B; domega_B; df_B; dmu_B; dphi];
x  = [ v_B;  omega_B;  f_B;  mu_B;  phi];
h  = [dv_B; f_B;  mu_B];
df_dx = jacobian(f, x);
dh_dx = jacobian(h, x);

model.I  = I_B;

matlabFunction(df_dx,'file','rigidBodyDynamicsDerivatives','vars',[x; dI; m; g]);
matlabFunction(dh_dx,'file','rigidBodyOutputsDerivatives','vars',[x; dI; m; g]);


