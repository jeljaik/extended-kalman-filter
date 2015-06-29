function y = measurement_withoutSkin(x,p,f_B_o_t,mu_B_o_t,f_B_c_t,mu_B_c_t)

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
if(isempty(f_B_o_t))
    f_B_o   = x(7:9  , 1);
    mu_B_o  = x(10:12, 1);
    f_B_c   = x(13:15, 1);
    mu_B_c  = x(16:18, 1);
else
    f_B_o = f_B_o_t;
    f_B_c = f_B_c_t;
    mu_B_o = mu_B_o_t;
    mu_B_c = mu_B_c_t;
end

phi     = x(19:21, 1);
B_R_G    = euler2dcm(phi);
m    = p.m;
G_g    = p.G_g;

dv_B     =          -(1/m)*S(omega_B) *(m*v_B) + (1/m) *( +f_B_o - f_B_c )+ B_R_G*G_g;

y  = [(dv_B-B_R_G*G_g); omega_B; f_B_o; mu_B_o; f_B_c; mu_B_c];
