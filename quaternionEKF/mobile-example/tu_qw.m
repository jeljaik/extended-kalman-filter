function [x, P] = tu_qw(x, P, omega, T, Rw)
%TU_QW(x, P, omega, T, Rw) This function implements the time update function based on the Dynamic
% model of the quaternion.
% x    : Orientation at the previous instant of time (4x1)
% omega: measured angular rate (3x1)
% T    : time since the last measurement.
% Rw   : process noise covariance matrix (3x1)
% 
% The discrete time update for a quaternion based on given angular
% velocities, \omega_k, and process noise w_k can be approximated with:
% 
% q_{k+1} = \exp{0.5*S(\omega_k + w_k)*T q_k
%         = (I + 0.5*S(\omega_k)*T)*q_k + T/2*\bar{S}(q_k)*w_k

x = (eye(4) + 0.5*Somega(omega*T))*x + T/2*Sq(x)*w_k;
