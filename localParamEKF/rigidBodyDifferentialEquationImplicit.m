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

v_B     = x(1:3  , 1);
omega_B = x(4:6  , 1); 
f_B_t   = x(7:9  , 1);
mu_B_t  = x(10:12, 1);
phi     = x(13:15, 1);

I_B  = p.I;
m    = p.m;
u    = p.u;
v    = p.v;
g    = p.g;

u_t  = [cos(t); sin(t); sin(2*t)]*u;
v_t  = [sin(t); cos(t); sin(2*t)]*v;

u_t    = u_t  -     v_B * u;
v_t    = v_t  - omega_B * v;

%% Rotation represented by phi
% phi = [alpha upsilon psi]'
alpha = phi(1);
upsilon=phi(2);
psi=phi(3);
R = euler2dcm(phi);

%% State equations

fLin     = - S(omega_B) * (m   * v_B    ) +  f_B_t + m*g.*R*[0; 0; 1];
fAng     = - S(omega_B) * (I_B * omega_B) +  mu_B_t;
df_B     =                                     u_t;
dmu_B    =                                     v_t;
dphi     = inv(R)*omega_B;

f        = [fLin; fAng; df_B; dmu_B; dphi];
