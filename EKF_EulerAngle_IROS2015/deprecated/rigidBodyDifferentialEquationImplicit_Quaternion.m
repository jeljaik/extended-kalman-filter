function f = rigidBodyDifferentialEquationImplicit_Quaternion(t,x,p)

% This piece of code computes the rigid body differential equations. The
% equation is defined as follows:
%
%      dq      = 1/2 omega_B . q
%
% m    dv^B    = -S(omega^B) (m       v^B) + f^B_1  + ... + f^B_n + mg
% 
% I^B domega^B = -S(omega^B) (I^B omega^B) + mu^B_1 + ... + mu^B_n
% 
% where q is the quaternion for representing the rotation matrix. 

v_B     = x(1:3  , 1);
omega_B = x(4:6  , 1); 
f_B_t   = x(7:9  , 1);
mu_B_t  = x(10:12, 1);
% q       = x(13:16, 1);

I_B  = p.I;
m    = p.m;
u    = p.u;
v    = p.v;
g    = p.g;

u_t  = [cos(t); sin(t); sin(2*t)]*u;
v_t  = [sin(t); cos(t); sin(2*t)]*v;

u_t    = u_t  -     v_B * u;
v_t    = v_t  - omega_B * v;

R    = q2dcm(q);

fLin     = - S(omega_B) * (m   * v_B    ) +  f_B_t + m*g.*R*[0; 0; 1];
fAng     = - S(omega_B) * (I_B * omega_B) +  mu_B_t;
df_B     =                                     u_t;
dmu_B    =                                     v_t;

f  = [fLin; fAng; df_B; dmu_B];
