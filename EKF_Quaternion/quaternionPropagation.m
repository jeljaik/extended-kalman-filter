function [ q ] = quaternionPropagation(b_hat_k, omega_hat_k, omega_m_1, tspan)
%QUATERNIONPROPAGATION Summary of this function goes here
%
%   B_HAT_K = 
%   OMEGA_HAT_K = 
%   OMEGA_M_1
%   TSPAN = [T0 TFINAL]
%   

  %% Propagation of the gyro bias
  b_hat_1 = b_hat_k;
  
  %% Estimate of the new turn rate 
  omega_hat_1 = omega_m_1 - b_hat_1;
  
  %% Quaternion propagation (q_bar_hat_1) with first order integrator
  % Recall that the first order integrator assumes a linear evolution of
  % omega during the ingration interval Delta_t
  
  % Define average turn rate with omega_hat_k and omega_hat_1
  omega_bar = 0.5*(omega_hat_1 + omega_hat_k)

  % Quaternion propagation
  q = [];
  for t=tspan(1):dt:tspan(2)
  q(end+1) = NormalizeV(exp(1/2*Omega(omega_bar)*gyroModel.dt) ...
                           + 1/48*(Omega(omega_hat_1)*Omega(omega_hat_k) ...
                           - Omega(omega_hat_k)*Omega(omega_hat_1))*gyroModel.dt^2)*q_hat_k;
  end

end

