function [f_Bo_t, mu_Bo_t, f_Bc_t, mu_Bc_t,omega_B] = rigidBodyInvDyn(Phi,dPhi, dPos,ddPos,tc,model, coeff_f, coeff_mu)
% RIGIDBODYINVDYN This function computes inverse dynamics for a rigid 
% body system with two external forces, assuming null linear velocity and 
% a desired orientation trajectory parametrized in ZYZ  Euler angles (roll,
% pitch and yaw).
%     
% It takes as inputs: 
% Phi   : Orientation as a time series vector
% Pos  : Position of the CoM in inertial frame expressed in body frame as
% time series vector
% model : Model structure containing mass (m), gravity (g), Inertia
%         matrix (I) and the integration time interval (dt).
% c_f12 : Forces ratio
% c_mu12: Torques ratio
%
% Authors: Jorhabib Eljaik,Naveen Kuppuswamy
% Istituto Italiano di Tecnologia
% Department of Robotics, Brain and Cognitive Sciences (RBCS)


    %% Retrieving function parameters
    m   = model.m;
    g   = model.g;
    I_B = model.I;
    dt  = model.dtInvDyn;

    %% Angular velocity and its derivative given a time-changing orientation 
    % expressed in ZYZ Euler angles
    %Phi      = Phi';

    % Derivative of the time-varying orientation
    %dPhi     = diff(Phi)/dt;
    T_t      = Tomega_dphi(Phi,'t');
    
    omega_B = zeros(size(Phi));
    domega_B = zeros(size(Phi));
    
    % Angular velocity for the desired time-varying orientation
    for i=1:length(dPhi)
        omega_B(i,:) = (T_t{i}*dPhi(i,:)')';
    end

     
    % Derivative of the angular velocity
    domega_B(1:end-1,1:3) = diff(omega_B)/dt;
    domega_B(end,:) = domega_B(end-1,:);


    %% Other quantities
    R = euler2dcm(Phi,'t');

    %% External wrenches to be applied
    % Forces
    %f_B1_t = [];
    %f_B2_t = [];
    f_t =zeros(3,length(tc));
    for i=1:length(tc)
       f_t(:,i)   = (m*ddPos(i,:)' + S(omega_B(i,:)')*m*dPos(i,:)' - m*g*R{i}*[0 0 1]');%-2/3*m*g*R{i}*[0 0 1]';
    end
    f_Bo_tt = coeff_f*f_t';
    f_Bc_tt = (1-coeff_f)*f_t';
    
    % Torques
    %mu_B1_t = [];
    %mu_B2_t = [];
    mu_t = zeros(3,length(tc));
    for i=1:length(tc)
        mu_t(:,i)  = I_B*domega_B(i,:)' + S(omega_B(i,:)')*I_B*omega_B(i,:)'; 
        %-2/3*(I_B*domega_B(:,:,i) + S(omega_B(:,:,i))*I_B*omega_B(:,:,i));
    end
    mu_Bo_tt = coeff_mu*mu_t';
    mu_Bc_tt = (1-coeff_mu)*mu_t';
   % t = tc;
    
    f_Bo_t = @(tDes)interp1(tc,f_Bo_tt,tDes)';
    f_Bc_t = @(tDes)interp1(tc,f_Bc_tt,tDes)';
    mu_Bo_t = @(tDes)interp1(tc,mu_Bo_tt,tDes)';
    mu_Bc_t = @(tDes)interp1(tc,mu_Bc_tt,tDes)';
    
   % df_B1_t = @(tDes)interp1(t,f_B1_tt,tDes);
   % df_B2_t = @(tDes)interp1(t,f_B1_tt,tDes);
  %  dmu_B1_t = @(tDes)interp1(t,f_B1_tt,tDes);
  %  dmu_B2_t = @(tDes)interp1(t,f_B1_tt,tDes);
    
    
end