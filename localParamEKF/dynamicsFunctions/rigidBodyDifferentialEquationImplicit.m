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
%persistent f_1_prev
%persistent f_2_prev
%persistent mu_1_prev
%persistent mu_2_prev
%persistent t_prev

%if(isempty(t_prev))
%    f_1_prev = 0; f_2_prev=0; 
%    mu_1_prev =0 ; mu_2_prev =0;
%    t_prev =0;
%end

v_B       = x(1:3  , 1);
omega_B   = x(4:6  , 1); 
%if ~useInvDyn
% f_B_t_1   = x(7:9  , 1);
% f_B_t_2   = x(10:12, 1);
% mu_B_t_1  = x(13:15, 1);
% mu_B_t_2  = x(16:18, 1);
    phi       = x(19:21, 1);
%else
%    phi  = x(19:21 , 1);
%end
    

I_B  = p.I;
m    = p.m;
% u    = p.u;
% ud   = p.ud;
% v    = p.v;
% vd   = p.vd;
g    = p.g;

% %% Input
% if ~useInvDyn
%     u_t  = [cos(t); sin(t); 0*sin(2* t)]*u/2;
%     v_t  = [sin(t); cos(t); 0*sin(2*t)]*v/2;
% 
%     u_t    = u_t  -     v_B * ud/2;
%     v_t    = v_t  - omega_B * vd/2;
% else
    %% Input2
    % Using first inverse dynamics to have appropriate input forces and torques
    % Interpolating data sets to obtain the value of the time-dependant
    % terms at the specified time. 

    

% end

%% Rotation represented by phi
% phi = [alpha upsilon psi]'
R = euler2dcm(phi);

%dt = t - t_prev;

%if(dt <= 0)
%    dt =1 ;
%end

    
%     f_B_t_1  = interp1(t_at_wrench, f_B1_tid, t)';
%     f_B_t_2  = interp1(t_at_wrench, f_B2_tid, t)';
%     mu_B_t_1 = interp1(t_at_wrench, mu_B1_tid, t)';
%     mu_B_t_2 = interp1(t_at_wrench, mu_B2_tid, t)';
% 
% 
% 
%     df_B_t_1  = interp1(t_at_wrench(1:end-1), diff(f_B1_tid,1,1)./repmat(diff(t_at_wrench(1:end))',1,3), t,'pchip')';
%     df_B_t_2  = interp1(t_at_wrench(1:end-1), diff(f_B2_tid,1,1)./repmat(diff(t_at_wrench(1:end))',1,3), t,'pchip')';
%     dmu_B_t_1 = interp1(t_at_wrench(1:end-1), diff(mu_B1_tid,1,1)./repmat(diff(t_at_wrench(1:end))',1,3), t,'pchip')';
%     dmu_B_t_2 = interp1(t_at_wrench(1:end-1), diff(mu_B2_tid,1,1)./repmat(diff(t_at_wrench(1:end))',1,3), t,'pchip')';
% 

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
%if ~useInvDyn
    dv_B     = - S(omega_B) * (m   * v_B    ) +  f_B_t_o + f_B_t_c + m*g.*R*[0; 0; 1];
    domega_B = - S(omega_B) * (I_B * omega_B) +  mu_B_t_o + mu_B_t_c;
    df_B_o   =  zeros(3,1); %df_B_t_1;%u_t;
    dmu_B_o  =  zeros(3,1); %dmu_B_t_1;%0.5*v_t;
    df_B_c   =  zeros(3,1); %df_B_t_2;%u_t;       %zeros(length(u_t),1);
    dmu_B_c  =  zeros(3,1); %dmu_B_t_2;%0.5*v_t;   %zeros(length(v_t),1)
    
%f_1_prev = f_B_t_1;
%f_2_prev = f_B_t_2;
%mu_1_prev =mu_B_t_1  ;
%mu_2_prev=mu_B_t_2  ;
%t_prev = t;
    
%else
 %   fLin     = - S(omega_B) * (m   * v_B    ) +  f_B_t_1 + f_B_t_2 + m*g.*R*[0; 0; 1];
 %   fAng     = - S(omega_B) * (I_B * omega_B) +  mu_B_t_1 + mu_B_t_2;
%     df_B_1   =   u_t;
%     df_B_2   =   u_t; %zeros(length(u_t),1);
%     dmu_B_1  =   0.5*v_t;
%     dmu_B_2  =   0.5*v_t; %zeros(length(v_t),1)    
%end

%dphi     =   pinv(R,1e-10)*omega_B;%inv(R)*omega

dphi = Tomega_dphi(phi)\omega_B;
%dphi     =   phiDerivative(phi,omega_B);%inv(R)*omega_B;
%
%if ~useInvDyn
    dxdt   = [dv_B; domega_B; df_B_o; dmu_B_o; df_B_c; dmu_B_c; dphi];
%else
 %   dxdt   = [fLin; fAng; dphi];
%en