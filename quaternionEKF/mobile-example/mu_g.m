function [x, P] = mu_g(x, P, yacc, Ra, g0)
%MU_G(x, P, yacc, Ra, g0) Accelerometer measurement update.
%   Measurements from the accelerometer can be modeled as:
% y^a_k : Q^T(q_k)(g^o  + F_k) + e^a_k
% x     : 
% yacc  : Acceleration at current time.
% Ra    : Measurement noise covariance matrix.
% g0    : Constant value of gravity acceleration.  

% Assuming no force F_k
x = Qq(x) *(g0) + Ra 

end

