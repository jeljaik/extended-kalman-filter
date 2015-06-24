function dxdt = rigidBodyDifferentialEquationImplicit(t, x, p, f_Bo_t, mu_Bo_t, f_Bc_t, mu_Bc_t, useInvDyn)
% RIGIDBODYDIFFERENTIALEQUATIONIMPLICIT Computes the rigid body differential equations.
%   RIGIDBODYDIFFERENTIALEQUATIONIMPLICIT(t, x, p, tc, f_B1_tc, mu_B1_tc,
%   f_B2_tc, mu_B2_tc).
%
%       dphi   = T_phi^-1 * omega
%
%       m dv^B = -S(omega^B) (m       v^B) + f^B_1  + ... + f^B_n + mg
% 
% I^B domega^B = -S(omega^B) (I^B omega^B) + mu^B_1 + ... + mu^B_n
% 
% phi    : ZYZ Euler angles representing orientation.
% T_phi  : Transformation matrix between omega and dphi

%% Arguments checking
if nargin < 8
    error('Not enough arguments for rigidBodyDifferentialEquationImplicit');
else
    if nargin < 9
        useInvDyn = 0;    
    else
        if nargin < 10
            useInvDyn = 1;
        else
            error('Too many input arguments for rigidBodyDifferentialEquationImplicit');
        end
    end
end

v_B       = x(1:3  , 1);
omega_B   = x(4:6  , 1); 

phi       = x(19:21, 1);

K_B       = x(22:30,1);    

K = reshape(K_B,3,3);

I_B  = p.I;
m    = p.m;

G_g    = p.G_g;



%% Rotation represented by phi
B_R_G = euler2dcm(phi);

if(isempty(f_Bo_t))
 f_B_t_o   = x(7:9  , 1);
 mu_B_t_o  = x(10:12, 1);
 f_B_t_c   = x(13:15, 1);
 mu_B_t_c  = x(16:18, 1);
else
    f_B_t_o = f_Bo_t(t);
    
    mu_B_t_o = mu_Bo_t(t);
    f_B_t_c = f_Bc_t(t);
    mu_B_t_c = mu_Bc_t(t);
end
    



%% 
dv_B     = - S(omega_B) * (m   * v_B    ) + (+f_B_t_o - f_B_t_c) + m.*B_R_G*G_g;
domega_B = - S(omega_B) * (I_B * omega_B) +  (+mu_B_t_o - mu_B_t_c) + (K*phi);

df_B_o   =  zeros(3,1); %df_B_t_1;%u_t;
dmu_B_o  =  zeros(3,1); %dmu_B_t_1;%0.5*v_t;
df_B_c   =  zeros(3,1); %df_B_t_2;%u_t;       %zeros(length(u_t),1);
dmu_B_c  =  zeros(3,1); %dmu_B_t_2;%0.5*v_t;   %zeros(length(v_t),1)
    

dphi = Tomega_dphi(phi)\omega_B;
dK = zeros(9,1);
dxdt   = [dv_B; domega_B; df_B_o; dmu_B_o; df_B_c; dmu_B_c; dphi;dK];
