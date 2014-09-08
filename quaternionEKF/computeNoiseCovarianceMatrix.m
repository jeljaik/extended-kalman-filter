function [ Qd ] = computeNoiseCovarianceMatrix( omega, sigma_r, sigma_w, dt, eps )
%COMPUTENOISECOVARIANCEMATRIX(OMEGA, SIGMA_R, SIGMA_W, DT, EPS) 
% Computes the covariance matrix of the noise in the discrete time system.
%
%   [INPUTS]
%   omega:   Turn rate estimate at the current time.
%   sigma_r: Noise covariance of the rate noise.
%   sigma_w: Noise covariance of the gyro bias.
%   dt:      Integration step
%   eps:     Tolerance for small omega.
%
%   [OUTPUTS]
%   Qd: Noise covariance matrix.

if(norm(omega) < eps)
    Q11 = sigma_r^2*dt*eye(3) ...
        + sigma_w^2*( eye(3)*dt^3/3 ...
            + 2*(dt)^5/factorial(5)*skew(omega)*skew(omega) );
    Q12 = -sigma_w^2*( eye(3)*dt^2/2 ...
                       - dt^3/factorial(3)*skew(omega) ...
                       + dt^4/factorial(4)*skew(omega)*skew(omega) );
else
    Q11 = sigma_r^2*dt*eye(3) ...
        + sigma_w^2*( eye(3)*dt^3/3 ...
                      + (1/norm(omega)^5)*( (norm(omega)*dt)^3/3 + 2*sin(norm(omega)*dt) - 2*norm(omega)*dt )*skew(omega)*skew(omega));
    Q12 = -sigma_w^2*( eye(3)*dt^2/2 ...
                       -(1/norm(omega)^3)*(norm(omega)*dt - sin(norm(omega)*dt))*skew(omega)...
                       +(1/norm(omega)^4)*(0.5*(norm(omega)*dt)^2 + cos(norm(omega)*dt)-1)*skew(omega)*skew(omega));
end

Q22 = sigma_w^2*dt*eye(3);

% Noise Covariance Matrix in block structure.
Qd = [Q11 Q22; Q12' Q22];

end

