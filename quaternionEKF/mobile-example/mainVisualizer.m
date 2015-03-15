%MAINVISUALIZER Visualizes the EKF-based orientation estimation of a phone
% with data recorded from its gyros, accelerometers and magnetometer.
%
% This script makes use of the following toolboxes:
% EKFUKF
% QUATERNION
% Author: Jorhabib Eljaik G.
% Istituto Italiano di Tecnologia
% iCub Facility

close all;
clear;
addpath(genpath('./utils'));
addpath(genpath('./ekfukf'));

% Flags
recordData = 0;
dummy_flag = 3;

if (recordData)
    %% Connecting to phone
    % Note: Orientation of the phone is as depicted here:
    % http://it.mathworks.com/help/releases/R2015a/supportpkg/mobilesensor/ug/phoneorv2.gif
    connector on;
    m = mobiledev;
    m.AngularVelocitySensorEnabled = 1;
    m.AccelerationSensorEnabled    = 1;
    m.OrientationSensorEnabled     = 1;
    m.Logging                      = 1;

    %% Retrieving data
    % Retrieving orientation [azimuth, pitch, roll], angular velocity and
    % linear acceleration.
    [orientation, t_orientation] = orientlog(m); % [degrees]
    [angVel     , t_angVel     ] = angvellog(m); % [rad/s]
    [accel      , t_accel      ] = accellog(m);  % [m/s^2]
else
    load('pitchTestGyroPlusAccel.mat');
end

%% Visualization
% 1: Debugging visualization
% 2: Visualizing batched orientation
% 3: Visualizing online orientation estimation
myView = visualizer('visualizer');

%% Test visualization. Mainly used for debugging axes orientation and such.
if dummy_flag == 1
    q_angleAxis.angle = 0.5;
    q_angleAxis.axis = [1;0;0];
    for i=1:100
        q = quaternion.angleaxis(q_angleAxis.angle, q_angleAxis.axis);
        q = q.double;
        setOrientation(myView, q);
        q_angleAxis.angle = q_angleAxis.angle + 0.1;
    end
end

%% Batch visualization. First collect data, then visualize it here. 
if dummy_flag == 2
    load('pitchTest.mat');
    for i=1:length(orientation)
        % The following mapping of the [azimuth, pitch, roll] readings from
        % the phone sensor define the local orientation frame on the phone
        % which has z (azimuth) pointing up, x to the right (-pitch) and y (roll) according
        % to the right hand rule. Angles are in DEGREES. Do no forget this
        % conversion before setting the orientation. A pitch test dataset
        % has been stored as 'pitchTest.mat'.
        q_quat = quaternion.eulerangles('xyz', pi/180*[-orientation(i,2); orientation(i,3); orientation(i,1)]);
        q_quat = q_quat.double; 
        setOrientation(myView, q_quat);
        pause(0.10);
    end
end

%% Filtering from recorded data
dt = 0.020;
if dummy_flag == 3
    % Interpolating data. Acceleration starts logging first.
    logTimes    = cell(1,3);
    logTimes{1} = t_orientation; 
    logTimes{2} = t_angVel;
    logTimes{3} = t_accel;
    
    min_t_orientation = min(logTimes{1});
    min_t_angVel      = min(logTimes{2});
    min_t_accel       = min(logTimes{3});
    mins              = [min_t_orientation  min_t_angVel  min_t_accel];
    [maxMins, idxMin] = max(mins);
    
    max_t_orientation = max(logTimes{1});
    max_t_angVel      = max(logTimes{2});
    max_t_accel       = max(logTimes{3});
    maxs              = [max_t_orientation max_t_angVel max_t_accel];
    [minMaxs, idxMax] = min(maxs);
    
    % Let's interpolate all the date between the min time of
    % logTimes{idxMin} and the max time of logTimes{idxMax}.
    % First Crop all signals
    newTime = maxMins:dt:minMaxs;
    interpOrientation = interp1(t_orientation, orientation, newTime);
    interpAngVel      = interp1(t_angVel, angVel, newTime);
    interpAccel       = interp1(t_accel, accel, newTime);
    
    quaternion_based_EKF;
end
