function [ P_1, q_bar_hat_1, b_hat_1, omega_hat_1 ] = ekfqEstimate(gyroModel, q_hat_k, b_hat_k, omega_m_1, omega_hat_k, P_k, eps)
%EKFQESTIMATE(gyroModel, q_hat_k, b_hat_k, omega_hat_k, omega_m_1, P_k, eps) 
% Propagation of the quaternion.
%  
%   INPUTS:
%   - gyroModel  : Gyroscope model information. For this function it
%                  provides the integration time dt and the noise 
%                  covariances sigma_r and sigma_w.
%   - q_hat_k    : Estimate of the quaternion at the previous time.
%   - b_hat_k    : Estimate of the gyroscope bias at the previous time
%                  step.
%   - omega_m_1  : Current turn rate measurement.
%   - omega_hat_k: Previous estimated turn rate which should have been
%                  computed as: omega_hat_k = omega_k - b_k.
%   - eps        : Tolerance value used in the computation of the
%                  transition matrix Phi.
%   - Pk         : Previous state covariance matrix from EKF.
%   
%   OUTPUTS:
%   - P_1        : New State Covariance Matrix for Extended Kalman Filter
  
  %% Checking arguments
  if nargin < 6
      error('**Not enough arguments**');
  end
  if nargin < 7
      eps = 0.001;
  end
  
  %% Propagation of the gyro bias
  b_hat_1 = b_hat_k;
  
  %% Estimate of the new turn rate 
  omega_hat_1 = omega_m_1 - b_hat_1;
  
  %% Quaternion propagation (q_bar_hat_1) with first order integrator
  % Recall that the first order integrator assumes a linear evolution of
  % omega during the ingration interval Delta_t
  
  % Define average turn rate with omega_hat_k and omega_hat_1
  omega_bar = 0.5*(omega_hat_1 + omega_hat_k);

  % Propagated quaternion
  q_bar_hat_1 = NormalizeV(exp(1/2*Omega(omega_bar)*gyroModel.dt) ...
                           + 1/48*(Omega(omega_hat_1)*Omega(omega_hat_k) ...
                           - Omega(omega_hat_k)*Omega(omega_hat_1))*gyroModel.dt^2)*q_hat_k;
  
  %% State Transition Matrix Phi
  Phi = computeTransitionMatrix(omega_hat_1, gyroModel.dt, eps);
  
  %% Noise Covariance Matrix
  Qd  = computeNoiseCovarianceMatrix(omega_hat_1, gyroModel.sigma_r, gyroModel.sigma_w, gyroModel.dt, eps);
  
  %% State Covariance Matrix for Extended Kalman Filter
  size(Phi)
  P_1 = Phi*P_k*Phi' + Qd;
  
end

