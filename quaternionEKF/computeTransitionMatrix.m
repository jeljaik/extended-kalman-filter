function [ Phi ] = computeTransitionMatrix( omega, dt, eps)
%COMPUTETRANSITIONMATRIX(OMEGA, DT, EPS) Computes the discrete state transition matrix for
%the implementation of the discrete time kalman filter equations.
%
%   [INPUTS]
%   omega: Turn rate estimate at current time.
%   dt:    Integration step.
%   eps:   Tolerance value for small omega.
%
%   [OUTPUTS]
%   Phi:   Discrete state transition matrix.
%   

  if norm(omega) < eps
      Theta = eye(3) - dt*skew(omega) + dt^2/2*skew(omega)*skew(omega);
      Psi = -eye(3)*dt + (dt^2/2)*skew(omega) - (dt^3/6)*skew(omega)*skew(omega);
  else
      Theta = cos(norm(omega)*dt)*eye(3)...
              - sin(norm(omega)*dt)*skew(omega/norm(omega)) ...
              + (1 - cos(norm(omega)*dt))*(omega/norm(omega))*(omega'/norm(omega));      
      Psi = -eye(3)*dt + (1/norm(omega)^2)*(1-cos(norm(omega)*dt))*skew(omega) ...
            - (1/norm(omega)^3)*(norm(omega)*dt - sin(norm(omega*dt))*skew(omega)*skew(omega));  
  end
  
  Phi = [Theta Psi; zeros(3,3) eye(3,3)];

end

