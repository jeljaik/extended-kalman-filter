function y = rigidBodyOutput(x,p,f_B_1_t,f_B_2_t,mu_B_1_t,mu_B_2_t)

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
if(isempty(f_B_1_t))
    f_B_1   = x(7:9  , 1);
    f_B_2   = x(10:12, 1);
    mu_B_1  = x(13:15, 1);
    mu_B_2  = x(16:18, 1);
else
    f_B_1 = f_B_1_t;
    f_B_2 = f_B_2_t;
    mu_B_1 = mu_B_1_t;
    mu_B_2 = mu_B_2_t;
end

mu_B_2_y = mu_B_2(2,:);
mu_B_2_z = mu_B_2(3,:);
f_B_2_x = f_B_2(1,:);

phi     = x(19:21, 1);
R    = euler2dcm(phi);
m    = p.m;
g    = p.g;

dv_B     =          -(1/m)*S(omega_B) *(m*v_B) + (1/m) *( f_B_1 + f_B_2 )+ g.*R*[0; 0; 1];
% domega_B =   I_B \ (-S(omega_B) * (I_B * omega_B) + mu_B_t);

%y  = [dv_B; omega_B; f_B_1; f_B_2; mu_B_1;mu_B_2];
y  = [dv_B; omega_B; f_B_1; mu_B_1; f_B_2_x; mu_B_2_y; mu_B_2_z];
%y  = [dv_B; f_B_1; f_B_2; mu_B_1;mu_B_2];

% y  = [f_B; mu_B];