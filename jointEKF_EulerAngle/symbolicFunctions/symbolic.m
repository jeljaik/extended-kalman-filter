clear all
close all
clc

syms I_B f_B_o mu_B_o f_B_c mu_B_c m real
syms I_Bxx I_Byy I_Bzz I_Bxy I_Byz I_Bxz g gRot1 gRot2 gRot3 gRot real
syms v_Bx v_By v_Bz real
syms f_B_ox f_B_oy f_B_oz real
syms f_B_cx f_B_cy f_B_cz real
syms omega_Bx omega_By omega_Bz real
syms mu_B_ox  mu_B_oy  mu_B_oz real
syms mu_B_cx  mu_B_cy  mu_B_cz real
syms phi1 phi2 phi3 real


% syms K_Bxx K_Bxy K_Bxz K_Byy K_Byz K_Bzz real
syms K_Bxx K_Bxy K_Bxz K_Byx K_Byy K_Byz K_Bzx K_Bzy K_Bzz real
% syms K_Bxx K_Byy K_Bzz real

I_B = [...
     I_Bxx I_Bxy I_Bxz;...
     I_Bxy I_Byy I_Byz;...
     I_Bxz I_Byz I_Bzz];
dI = [I_Bxx I_Bxy I_Bxz I_Byy I_Byz I_Bzz ]';
 


% K_B =     diag([K_Bxx; K_Byy; K_Bzz]);
% K_B = [ K_Bxx K_Bxy K_Bxz;...
%         K_Bxy K_Byy K_Byz;...
%         K_Bxz K_Byz K_Bzz ];


K_B = [ K_Bxx K_Bxy K_Bxz;...
        K_Byx K_Byy K_Byz;...
        K_Bzx K_Bzy K_Bzz ];


w = [K_Bxx; K_Bxy; K_Bxz; K_Byx; K_Byy; K_Byz; K_Bzx; K_Bzy; K_Bzz];
% w = [K_Bxx; K_Byy; K_Bzz; K_Bxy; K_Bxz; K_Byz];
% w = [K_Bxx K_Byy K_Bzz]';

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
gRot = [gRot1;gRot2;gRot3];

dv_B     = -S(omega_B) * v_B - 1/m * f_B_o + 1/m*f_B_c - g.*R*gRot;
domega_B =  I_B \ (-S(omega_B) * (I_B * omega_B) - mu_B_o + mu_B_c + (K_B * phi));
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

PHI = kron(phi.',eye(3));
dmu_B_s = S(domega_B) * (I_B * omega_B) + S(omega_B) * (I_B * domega_B) - K_B * dphi + dmu_B_o - dmu_B_c;
dK = pinv(PHI) * dmu_B_s;

h_imu = [(dv_B + g.*R*gRot); omega_B];
%h_imu = [R'*dv_B ; omega_B];
h_fto = [f_B_o; mu_B_o];
h_ftc = [f_B_c; mu_B_c];
h_skin = f_B_cz;
%h_skin = [f_B_c_x; mu_B_cy; mu_B_cz];

f     = [dv_B; domega_B; df_B_o;  dmu_B_o;df_B_c; dmu_B_c; dphi];
x     = [ v_B;  omega_B;  f_B_o;  mu_B_o; f_B_c;  mu_B_c;  phi];

%dynParam     = [dK(1); dK(5); dK(9); dK(2); dK(3); dK(6)];
dynParam = dK;

%h_noSkin_noGyro = [dv_B; f_B_1; f_B_2; mu_B_1; mu_B_2];
%h = [dv_B; omega_B; f_B_1; mu_B_1; f_B_2x; mu_B_2y;mu_B_2z];
%h_noSkin_withGyro = [dv_B; omega_B; f_B_1; f_B_2; mu_B_1; mu_B_2];

h = [h_imu ; h_fto; h_ftc; h_skin];

df_dx = jacobian(f, x);
df_dw = jacobian(f,w);
dk_dw = jacobian(dynParam,w); 

F = [df_dx df_dw;zeros(9,21) dk_dw];

%dh_dx_noSkin_noGyro = jacobian(h_noSkin_withGyro, x);
%dh_dx_withSkin_withGyro = jacobian(h_withSkin_withGyro,x);

dh_dx = jacobian(h,x);
dh_dw = jacobian(h,w);

H = [dh_dx dh_dw];

model.I  = I_B;

% matlabFunction(F,'file','./symbolic/rigidBodyDynamicsDerivatives','vars',[x; w; dI; m; g]);

% matlabFunction(H,'file','./symbolic/rigidBodyOutputsDerivatives','vars',[x; w; dI; m; g]);
matlabFunction(F,'file','./symbolicFunctions/rigidBodyDynamicsDerivatives','vars',[x; w; dI; m; g; gRot]);

matlabFunction(H,'file','./symbolicFunctions/rigidBodyOutputsDerivatives','vars',[x; w; dI; m; g; gRot]);
