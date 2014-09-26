function [f_B1_t, mu_B1_t, f_B2_t, mu_B2_t] = rigidBodyInvDyn(Phi, model, c_f12, c_mu12)
% RIGIDBODYINVDYN This function computes inverse dynamics for a rigid 
% body system with two external forces, assuming null linear velocity and 
% a desired orientation trajectory parametrized in ZYZ  Euler angles (roll,
% pitch and yaw).
%     
% It takes as inputs: 
% Phi   : Orientation as a time series vector
% model : Model structure containing mass (m), gravity (g), Inertia
%         matrix (I) and the integration time interval (dt).
% c_f12 : Forces ratio
% c_mu12: Torques ratio
%
% Author: Jorhabib Eljaik
% Istituto Italiano di Tecnologia
% Department of Robotics, Brain and Cognitive Sciences (RBCS)


    %% Retrieving function parameters
    m   = model.m;
    g   = model.g;
    I_B = model.I;
    dt  = model.dt;

    %% Angular velocity and its derivative given a time-changing orientation 
    % expressed in ZYZ Euler angles
    Phi      = Phi';

    % Derivative of the time-varying orientation
    dPhi     = diff(Phi)/dt;
    T_t      = Tomega_dphi(Phi,'t');

    % Angular velocity for the desired time-varying orientation
    for i=1:length(dPhi)
        omega_B(:,1,i) = T_t{i}*dPhi(i,:)';
    end

    % Derivative of the angular velocity
    domega_B = diff(omega_B,1,3)/dt;


    %% Other quantities
    R = euler2dcm(Phi,'t');

    %% External wrenches to be applied
    % Forces
    f_B1_t = [];
    f_B2_t = [];
    for i=1:length(domega_B)
        f_B1_t(end+1,:)   = -2/3*m*g*R{i}*[0 0 1]';
    end
    f_B2_t = c_f12*f_B1_t;
    
    % Torques
    mu_B1_t = [];
    mu_B2_t = [];
    for i=1:length(domega_B)
        mu_B1_t(end+1,:)  = -2/3*(I_B*domega_B(:,:,i) + S(omega_B(:,:,i))*I_B*omega_B(:,:,i));
    end
    mu_B2_t = c_mu12*f_B2_t;
    
end