%EKFQUAT_PROPAGATION1 Propagation phase of the Extended Kalman Filter for
%quaternions.
%   In this phase the filter returns a prediction of the attitude based on
%   the last estimate and proprioceptive measurements. In this particular
%   case, instead of using the model of the system, the data from the IMU
%   is used as dynamic model replacement. To this end we will use the model
%   of gyroscope that relates the measured turn rate with the real angular
%   velocity as omega_m = omega + b + n_r

function [ output_args ] = ekfQuat_propagation1( input_args )


end

