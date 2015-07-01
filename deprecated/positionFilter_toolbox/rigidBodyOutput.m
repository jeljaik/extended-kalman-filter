function y = rigidBodyOutput(x,p)

% This piece of code computes the rigid body differential equations. The
% equation is defined as follows:
%
% m    dv^B    + S(omega^B) (m       v^B) = f^B_1  + ... + f^B_n + mg
% 
% I^B domega^B + S(omega^B) (I^B omega^B) = mu^B_1 + ... + mu^B_n
% 
% and therefore:
%
%      dv^B = - S(omega^B) (        v^B) + 1/m f^B_1  + ... + 1/m f^B_n + g
% 
%  domega^B = - inv(I^B) (S(omega^B) (I^B omega^B) + mu^B_1 + ... + mu^B_n)

v_B     = x(1:3, 1);
omega_B = x(4:6, 1); 
f_B     = x(7:9  , 1);
mu_B    = x(10:12, 1); 
q       = x(13:16, 1);

R    = q2dcm(q);
m    = p.m;
g    = p.g;

dv_B     =          -S(omega_B) * v_B + 1/m * f_B + g.*R*[0; 0; 1];
% domega_B =   I_B \ (-S(omega_B) * (I_B * omega_B) + mu_B_t);

y  = [dv_B; f_B; mu_B];
% y  = [f_B; mu_B];