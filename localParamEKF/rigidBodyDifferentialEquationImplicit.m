function dxdt = rigidBodyDifferentialEquationImplicit(t, x, p, t_invDyn, f_B1_tid, mu_B1_tid, f_B2_tid, mu_B2_tid, useInvDyn)
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
if ~useInvDyn
    f_B_t_1   = x(7:9  , 1);
    f_B_t_2   = x(10:12, 1);
    mu_B_t_1  = x(13:15, 1);
    mu_B_t_2  = x(16:18, 1);
    phi       = x(19:21, 1);
else
    phi  = x(7:9 , 1);
end
    

I_B  = p.I;
m    = p.m;
u    = p.u;
ud   = p.ud;
v    = p.v;
vd   = p.vd;
g    = p.g;

%% Input
if ~useInvDyn
    u_t  = [cos(t); sin(t); 0*sin(2* t)]*u/2;
    v_t  = [sin(t); cos(t); 0*sin(2*t)]*v/2;

    u_t    = u_t  -     v_B * ud/2;
    v_t    = v_t  - omega_B * vd/2;
end

%% Input2
% Using first inverse dynamics to have appropriate input forces and torques
% Interpolating data sets to obtain the value of the time-dependant
% terms at the specified time. 
if useInvDyn
    f_B_t_1  = interp1(t_invDyn, f_B1_tid, t)';
    f_B_t_2  = interp1(t_invDyn, f_B2_tid, t)';
    mu_B_t_1 = interp1(t_invDyn, mu_B1_tid, t)';
    mu_B_t_2 = interp1(t_invDyn, mu_B2_tid, t)';
end

%% Rotation represented by phi
% phi = [alpha upsilon psi]'
R = euler2dcm(phi);

%% 
if ~useInvDyn
    fLin     = - S(omega_B) * (m   * v_B    ) +  f_B_t_1 + f_B_t_2 + m*g.*R*[0; 0; 1];
    fAng     = - S(omega_B) * (I_B * omega_B) +  mu_B_t_1 + mu_B_t_2;
    df_B_1   =   u_t;
    df_B_2   =   u_t;       %zeros(length(u_t),1);
    dmu_B_1  =   0.5*v_t;
    dmu_B_2  =   0.5*v_t;   %zeros(length(v_t),1)
else
    fLin     = - S(omega_B) * (m   * v_B    ) +  f_B_t_1 + f_B_t_2 + m*g.*R*[0; 0; 1];
    fAng     = - S(omega_B) * (I_B * omega_B) +  mu_B_t_1 + mu_B_t_2;
%     df_B_1   =   u_t;
%     df_B_2   =   u_t; %zeros(length(u_t),1);
%     dmu_B_1  =   0.5*v_t;
%     dmu_B_2  =   0.5*v_t; %zeros(length(v_t),1)    
end

dphi     =   inv(R)*omega_B;

if ~useInvDyn
    dxdt   = [fLin; fAng; df_B_1; df_B_2; dmu_B_1; dmu_B_2; dphi];
else
    dxdt   = [fLin; fAng; dphi];
end
