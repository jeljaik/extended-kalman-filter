function [ mu1, sigma1, w ] = ekf( mu0, sigma0, u1, z, errZ, sigr, sigw, dt)
% EKF with quaternions as seen in
% 'Indirect Kalman Filter for 3D Attitude Estimation'
% by Trawny and Roumeliotis
% Author: Olivier Dugas, ing. jr
 
%% Initial Setup
% Assume we receive gyro measurements w0 and w1. 
w0 = u1(:,1);
w1 = u1(:,2);
% We have an estimate of the quaternion q_hat_avg and the bias b0
% mu0 = [q_hat_avg_0 ; b0]
q_hat_avg_0 = mu0(1:4);
b0 = mu0(5:7);
% Together with the corresponding covariance matrix P_k|k
% sigma0 = P_k|k
% From this, we also have an estimate of the old turn rate w_hat_0
w_hat_0 = w0 - b0;
 
%% State Prediction
% Regular EKF algorithm line equivalent: mu1_ = g(u1, mu0)
 
% We instead proceed as follows:
% 1. We propagate the bias (assuming the bias is constant over the integration interval)
b1 = b0; % TODO : Here is a problem : we have no way of validating or updating the bias. Update it externally periodically can do the trick.
 
% 2. Using the measurement w1 and b1, we form the estimate of the new turn rate w_hat_1
w_hat_1 = w1 - b1;
 
% 3. We propagate the quaternion using a first order integrator with w_hat_0 and w_hat_1 to obtain q_hat_avg_1
w_avg = (w_hat_0 + w_hat_1) / 2;
q_hat_avg_1 = NormalizeV((exp(1/2*Omega(w_avg)*dt) + 1/48*(Omega(w_hat_1)*Omega(w_hat_0) - Omega(w_hat_0)*Omega(w_hat_1))*dt^2) * q_hat_avg_0);
% q_hat_avg_1
% mu1_ = [q_hat_avg_1 ; b1]
 
%% Error Propagation during State Prediction
% Regular EKF algorithm line equivalent: sigma1_ = G1*sigma0*G1' + R1
 
% 4. We compute the state transition matrix Phi and the discrete time noise covariance matrix Qd
% Computation of Phi
% NOTE: IN THE FOLLOWING EQUATIONS IT IS NOT CLEAR WHY THE AUTHOR USES
% w_avg INSTEAD OF w_hat_0
if norm(w_avg) < 0.00001 %Too small, may induce numerical instability. Estimating instead.
    Theta = eye(3) - dt*skew(w_avg) + ((dt.^2)/2)*(skew(w_avg)*skew(w_avg));
    Psi = -eye(3)*dt + ((dt^2)/2)*skew(w_avg) - ((dt^3)/6)*(skew(w_avg)*skew(w_avg));
else
    Theta = cos(norm(w_avg)*dt)*eye(3) - sin(norm(w_avg)*dt)*skew(w_avg/norm(w_avg)) + (1 -cos(norm(w_avg)*dt))*(w_avg/norm(w_avg))*(w_avg'/norm(w_avg));
    Psi = -eye(3)*dt + (1/norm(w_avg).^2)*(1-cos(norm(w_avg)*dt))*skew(w_avg) - (1/norm(w_avg).^3)*(norm(w_avg)*dt - sin(norm(w_avg)*dt))*(skew(w_avg)*skew(w_avg));
end
Phi = [Theta Psi; zeros(3) eye(3)];
 
% Computation of Qd
if norm(w_avg) < 0.00001  %Too small, may induce numerical instability. Estimating instead.
    Q11 = (sigr.^2)*dt*eye(3) + (sigw.^2)*(eye(3)*dt^3/3 + (dt^5/60)*(skew(w_avg)*skew(w_avg)));
    Q12 = -(sigw^2) * ( eye(3)*dt.^2/2 - (dt^3/6)*skew(w_avg) + (dt^4/24)*(skew(w_avg)*skew(w_avg)));
else
    Q11 = (sigr.^2)*dt*eye(3) + (sigw.^2)*( eye(3)*dt.^3/3 + (((norm(w_avg)*dt)^3/3 + 2*sin(norm(w_avg)*dt) - 2*norm(w_avg)*dt )/ (norm(w_avg)^5))*(skew(w_avg)*skew(w_avg)));
    Q12 = -(sigw^2) * ( eye(3)*dt.^2/2 - ((norm(w_avg)*dt - sin(norm(w_avg)*dt))/(norm(w_avg)^3))*skew(w_avg) + (((norm(w_avg)*dt)^2/2 + cos(norm(w_avg)*dt) - 1)/(norm(w_avg)^4))*(skew(w_avg)*skew(w_avg)));
end
Q22 = (sigw^2)*dt*eye(3);
Qd = [Q11 Q12; Q12' Q22];
 
% 5. We Compute the state covariance matrix according to the Extended Kalman Filter equation
sigma1_ = Phi*sigma0*Phi' + Qd;
 
%% Computation of the Kalman gain
% Regular EKF algorithm line equivalent: K1 = sigma1_*H1'/(H1*sigma1_*H1' + errZ)

% NOTE: WHAT SHOULD WE USE AS ATTITUE SENSOR?? THREE-AXIS MAGNETOMETER?
% Measurement prediction function: we convert q_hat_avg_1 to an axis-angle representation
z_hat = convertToAxisAngle(q_hat_avg_1); 
% z_hat = q_hat_avg_1; 
 
% 6. Compute the measurement matrix H
% H = [skew(z_hat) zeros(3)]; 
H = [skew(q_hat_avg_1) zeros(3)]; 
 
% 7. Compute residual r according to r = z - z_hat
z_quat = convertToQuaternion(z);
z_hat_quat = convertToQuaternion(z_hat);
% NOTE: THE ORIGINAL PAPER SAYS THIS SHOULD BE A PROPER DIFFERENCE NOT A
% MULTIPLICATION ?? r = z - z_hat
r = multiplyQuaternion(z_quat,invQuat(z_hat_quat));
% r = multiplyQuaternion(z,invQuat(z_hat))
 
% 8. Compute the covariance of the residual S as
S = H*sigma1_*H' + errZ;
 
% 9. Compute the Kalman gain K
K = sigma1_*(H'*inv(S));
 
%% Measurement Update - State
% Regular EKF algorithm line equivalent: mu1 = mu1_ + K1*(z1-h(mu1_))
 
% 10. Compute the correction deltaX = [2*dq;db]
deltaX = K*r(1:3);
% NOTE: FROM THE PAPER THESE VARIABLES ARE ACTUALLY dq_hat AND db_hat
dq = deltaX(1:3)/2;
db = deltaX(4:6);
 
% 11. Update the quaternion
if (dq'*dq) > 1
    dq_hat_avg_1 = (1/sqrt(1 + (dq'*dq))) * [dq ; 1];
else
    dq_hat_avg_1 = [dq ; (sqrt(1 - (dq'*dq)))];
end

q = multiplyQuaternion(dq_hat_avg_1, q_hat_avg_1);
 
% 12. Update the bias
b = b1 + db;
 
mu1 = [q ; b];
 
% 13. Update the estimated turn rate using the new estimate for the bias
w = w1 - b;
 
%% Measurement Update - Error
% Regular EKF algorithm line equivalent: sigma1 = (eye(6) - K1*H1)*sigma1_
 
% 14. Compute the new updated Covariance matrix
sigma1 = (eye(6) - K*H) * sigma1_ * (eye(6) - K*H)' + K*errZ*K';
 
end