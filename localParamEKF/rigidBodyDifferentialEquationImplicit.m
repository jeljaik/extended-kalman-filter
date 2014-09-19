function f = rigidBodyDifferentialEquationImplicit(t,x,p)

% This piece of code computes the rigid body differential equations. The
% equations are defined as:
%
%       dphi   = T_phi^-1 * omega
%
% m    dv^B    = -S(omega^B) (m       v^B) + f^B_1  + ... + f^B_n + mg
% 
% I^B domega^B = -S(omega^B) (I^B omega^B) + mu^B_1 + ... + mu^B_n
% 
% phi    : ZYZ Euler angles representing orientation.
% T_phi  : Transformation matrix between omega and dphi

v_B       = x(1:3  , 1);
omega_B   = x(4:6  , 1); 
f_B_t_1   = x(7:9  , 1);
f_B_t_2   = x(10:12, 1);
mu_B_t_1  = x(13:15, 1);
mu_B_t_2  = x(16:18, 1);
phi       = x(19:21, 1);

I_B  = p.I;
m    = p.m;
u    = p.u;
ud   = p.ud;
v    = p.v;
vd   = p.vd;
g    = p.g;

u_t  = [cos(t); sin(t); 0*sin(2* t)]*u/2;
v_t  = [sin(t); cos(t); 0*sin(2*t)]*v/2;

u_t    = u_t  -     v_B * ud/2;
v_t    = v_t  - omega_B * vd/2;

%% Rotation represented by phi
% phi = [alpha upsilon psi]'
R = euler2dcm(phi);

%% 
fLin     = - S(omega_B) * (m   * v_B    ) +  f_B_t_1 + f_B_t_2 + m*g.*R*[0; 0; 1];
fAng     = - S(omega_B) * (I_B * omega_B) +  mu_B_t_1 + mu_B_t_2;
df_B_1   =   u_t;
df_B_2   =   u_t; %zeros(length(u_t),1);
dmu_B_1  =   0.5*v_t;
dmu_B_2  =   0.5*v_t; %zeros(length(v_t),1)
dphi     =   inv(R)*omega_B;

f        = [fLin; fAng; df_B_1; df_B_2; dmu_B_1; dmu_B_2; dphi];
