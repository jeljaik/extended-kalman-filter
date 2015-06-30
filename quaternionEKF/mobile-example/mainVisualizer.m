%MAINVISUALIZER Visualizes the EKF-based orientation estimation of a phone
% with data recorded from its gyros, accelerometers and magnetometer.
%
% e.g. mainVisualizer('debug'   , [], [1 0 0])
%      mainVisualizer('batchEKF', 'pitchTestGyroPlusAccel.mat')
%      mainVisualizer('batchVis', 'pitchTest.mat')
%      mainVisualizer('onlineEKF')
%
% In:
% TYPE        'debug' for debugging axes. With this choice you have to
%             specify also 'revAxis', for the axis you want to debug as a
%             unit vector, e.g. [1 0 0].
% DATASET     String with the name of the dataset to load when type is
%             'batchEKF' or 'batchVis'.
% REVAXIS     Vector specifying the axis of rotation for testing the
%             visualizer. Use when type = 'debug' only.
%
% This script makes use of the following toolboxes:
% EKFUKF by ...
% QUATERNION by ...
%
% Author: Jorhabib Eljaik G.
% Istituto Italiano di Tecnologia
% iCub Facility. 2015

function [Mstored] = mainVisualizer(type, dataset, revAxis)
% dummy_flag = 1: Debugging visualization
% dummy_flag = 2: Visualizing batched orientation
% dummy_flag = 3: Visualizing online orientation estimation
close all;
addpath(genpath('./utils'));
addpath(genpath('./ekfukf'));

if (strcmp(type,'debug') && isempty(revAxis))
    error('When type is "debug", "revAxis" must be specified, e.g. [1 0 0]');
end
if ( (strcmp(type,'batchVis') || strcmp(type,'batchEKF')) && isempty(dataset))
    error('When  type is "batchVis" or "batchEKF", the value of "dataset" must be specified');
end

if (strcmp(type,'debug'))
    dummy_flag = 1;
    xDebug = revAxis(1);
    yDebug = revAxis(2);
    zDebug = revAxis(3);
else
    if (strcmp(type, 'batchVis'))
        dummy_flag = 2;
    else
        if (strcmp(type,'batchEKF'))
            dummy_flag = 3;
        end
        if(strcmp(type,'onlineEKF'))
            dummy_flag = 4;
        end
    end
end

% Flags
recordData = 0;
myView = visualizer2('visualizer');

%% Test visualization. Mainly used for debugging axes orientation and such.
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

if dummy_flag == 1
    q_angleAxis.angle = 0.5;
    q_angleAxis.axis = [xDebug;yDebug;zDebug];
    for i=1:100
        q = quaternion.angleaxis(q_angleAxis.angle, q_angleAxis.axis);
        q = q.double;
        setOrientation(myView, q);
        q_angleAxis.angle = q_angleAxis.angle + 0.1;
    end
end

%% Batch visualization. First collect data, then visualize it here.
if dummy_flag == 2
    load(dataset);
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
dt = 0.010;
if dummy_flag == 3
    load(dataset);
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
    
    quaternion_based_EKF(interpOrientation, interpAccel, interpAngVel, dt, newTime);
end

%% Online filtering and visualization
if dummy_flag == 4
    disp('Make sure your Matlab Mobile application has been launched on your Android device and that you have setup the right IP address');
    % Create interface
    connector on;
    m = mobiledev;
    loops = 200;
    
    if(m.Connected)
        disp('Phone connected to MATLAB');
        
        % Activate all sensors
        m.AngularVelocitySensorEnabled = 1;
        m.AccelerationSensorEnabled    = 1;
        m.OrientationSensorEnabled     = 1;
        % Start logging data
        m.Logging = 1;
        
        currAcc = 0;
        currAngVel = 0;
        currOrient = 0;
        
        while(isempty(m.Acceleration) && isempty(m.AngularVelocity) && isempty(m.Orientation))
            disp('Waiting for data ...');
        end
        
        disp('Acceleration before entering loop...');
        disp(m.Acceleration);
        disp('Angular velocity before entering loop...');
        disp(m.AngularVelocity);
        
        % Kalman dt
        dt = 0.03;

        % Gyro covariance matrix
        stdGyro = 0.001;
        Qgyro = stdGyro*eye(3);
        
        % Measurement noise covariance
        measCov = 0.00001;
        R = measCov*eye(4);

        param{1} = dt;
        % Param{2} is gravity in g units. This is convenient for the quaternion
        % representation. MEASUREMENTS SHOULD BE NORMALIZED THEN.
        param{2} = [0 0 1]';
        param{3} = measCov;
        
        % Prior on the process covariance matrix
        PP  = 1*eye(4);
        
        % Initial orientation
        MM = quaternion.eulerangles('xyz', pi/180*[0 0 0]');
        MM = MM.double;
        
        %
        idx = 1;
        Mstored = [];
%         quat_fig = figure;
        startTime = tic;            
        while(1) 
            %% WAITING FOR BOTH MEASUREMENTS TO ARRIVE
            while(isempty(m.Acceleration))
%                 disp('waiting for acc...');
            end
            currAcc = m.Acceleration;
            while(isempty(m.AngularVelocity))
%                 disp('waiting for angVel...');
            end
            currAngVel = m.AngularVelocity;
            
            %% SENDING ACC. AT A DIFFERENT RATE FROM THE GYRO
            m.Acceleration 
            
            
            dt = toc(startTime);
            [MM, PP] = online_quaternion_based_EKF([],currAcc, currAngVel, dt, MM, PP, Qgyro, R, param, idx);
            startTime = tic;
            setOrientation(myView, MM);
%             PlotRotation(quaternion(MM), []);
%             loops = loops - 1;
%             Mstored = [Mstored, MM];
        end
        
    else
        disp('Phone not connected');
    end
    
    
    m.Logging                      = 0;
    m.AngularVelocitySensorEnabled = 0;
    m.AccelerationSensorEnabled    = 0;
    m.OrientationSensorEnabled     = 0;
    
    clear m;
    connector off;
end

end