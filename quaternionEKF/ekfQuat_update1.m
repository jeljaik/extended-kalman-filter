%EKFQUAT_UPDATE1 Extended Kalman Filter update step with quaternions.
%   Based on 'Indirect Kalman Filter for 3D Attitude Estimation'
%   by Trawny N. and Roumeliotis S.
%   Author: Jorhabib Eljaik G. 
%   Istituto Italiano di Tecnologia 
%   Department of Robotics, Brain and Cognitive Sciences.
%
%   EKFQUAT_UPDATE1 Computes the update phase of the Extended Kalman Filter
%   for quaternions, given the propagated state estimates as well as their
%   covariance matrix, the current measurement and the measurement matrix.
%   The update is computed as follows:
%   1. Compute the measurement matrix
%   2. Compute residual according to r = z(x)z_hat
%   3. Compute the covariance of the residual S as S = H*P*H' + R
%   4. Compute the Kalman gain K as K = P*H'*S^-1
%   5. Compute the correction Dx_h = K*r
%   6. Update the quaternion as: q_hat_1 = \delta q_hat (x) q_hat_1_k
%   7. Update the bias b_hat_1 = b_hat_1_k + \Delta b_hat
%   8. Update the estimated turn rate using the new estimate for the bias
%       as: omega_hat_1 = omega_m_1 - b_hat_1
%   9. Compute the new updated Covariance matrix as:
%       P_1 = (I - K*H)*P_1_k*(I - K*H)' + K*R*K'

function [ output_args ] = ekfQuat_update1( input_args )
    
end

