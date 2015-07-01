function [yMeasSim,model] = simulatedMeasurement(tKalman,R,model,forceSim,plots)


% Department of Robotics, Barin and Cognitive Sciences
% Istituto Italiano di Tecnologia, 11 September 2014
%
% Authors : Naveen Kuppuswamy
% 
% This piece of code simulates the problem of estimating dynamic quantities
% for a single rigid body with distributed force/torque measurements and
% distributed gyro/accelerometers measurements. The motion is governed by
% the following differential equation with a local parametrization of the
% orientation in ZYZ Euler angles.
%
% m    dv^B    + S(omega^B) (m       v^B) = f^B_1  + ... + f^B_n + mg^B
%
% I^B domega^B + S(omega^B) (I^B omega^B) = mu^B_1 + ... + mu^B_n
%
%                                  dphi   = T_phi^-1 * omega
%
% where we defined the following quantities:
%
% I^B    : inertia in the body reference frame
% m      : mass of the rigid body
% f^B_i  : i-th force expressed in the body reference frame
% mu^B_i : i-th torque expressed in the body reference frame
% omega^B: angular velocity expressed in the body reference frame
% v^B    : linear velocity in the body reference frame
% phi    : ZYZ Euler angles representing orientation.
% T_phi  : Transformation matrix between omega and dphi

    T = tKalman(end);
    disp('Using simulated data');


    if(exist(sprintf('./data/sim_%d.mat',round(1000*T)),'file')~=2 || ~isempty(forceSim) )

        disp('Computing Inverse Dynamics'); drawnow();
        %% Inverse Dynamics - Planning
        % This section computes the external force and torque datasets used in
        % order to have a desired orientation away from singularities. 
        useInvDyn = 'y';
        
        [f_B1_tid, mu_B1_tid, f_B2_tid, mu_B2_tid, model.x0] = InverseDynamics(T,model, plots);

        %model.th_init = th_0;
        %R0        = [cos(model.th_init) -sin(model.th_init) 0; sin(model.th_init) cos(model.th_init) 0; 0 0 1];
        %model.x0  = [0*ones(6,1); 0*ones(12,1);0*ones(12,1); dcm2euler(R0)];

        %% Forward Dynamics - Simulation
        disp('Integrating Forward Dynamics'); drawnow();

        %if(useInvDyn == 'y')
            %model.x0 = [0*ones(6,1); dcm2euler(R0)];
        %    model.x0  = [dP0;omega_B0; f_B1_tid(1,:)';f_B2_tid(1,:)';mu_B1_tid(1,:)';mu_B2_tid(1,:)'; th_0];
        %end
        [tForDyn,   xForDyn] = integrateForwardDynamics(model.x0, model, 0:model.dtForDyn:T,f_B1_tid, mu_B1_tid, f_B2_tid, mu_B2_tid, useInvDyn);

        % Let's compute the output used in the Kalman filter. In this specific
        % case we will use dv^B, omega^B f^B and mu^B. In practice:
        %
        % y = [dv^B, omega^B, f^B_1, f^B_2, mu^B_1, mu^B_2];    disp('Adding measurement noise, preparing for kalman filter');

        yForDyn = zeros(18,length(tForDyn));

        for i = 1:length(tForDyn)
           yForDyn(:,i) = rigidBodyOutput(xForDyn(i,:)',model,...
           f_B1_tid(tForDyn(i)),f_B2_tid(tForDyn(i)),...
           mu_B1_tid(tForDyn(i)),mu_B2_tid(tForDyn(i)));
        end
        yForDyn = yForDyn';
            
        % Augmenting state with input forces and torques
        xForDyn(:,7:18) = [f_B1_tid(tForDyn)',f_B2_tid(tForDyn)',...
               mu_B1_tid(tForDyn)',mu_B2_tid(tForDyn)'];

        if plots
            figure(2);
            subplot(3,2,2);
            plot(tForDyn,xForDyn(:,1:3)); hold on; axis tight;
            xlabel('time t(sec)');
            ylabel('Velocity (m/sec)');
            title('Obtained Motion');
            subplot(3,2,4);
            plot(tForDyn,xForDyn(:,4:6)); hold on; axis tight;
            xlabel('time t(sec)');
            ylabel('AngularVelocity (rad/sec)');

            subplot(3,2,6);
            plot(tForDyn,yForDyn(:,1:3)); hold on;axis tight;
            xlabel('time t(sec)');
            ylabel('Acceleration (m/sec^2)');
           drawnow();
        end

        if(exist('data','dir')==0)
            mkdir('.','data');
        end

    %% saving simulation trial in file
        save(sprintf('./data/sim_%d',round(1000*T)),'tForDyn','yForDyn','xForDyn','model');
    else
        disp('Using stored simulation data'); drawnow();
        
        load(sprintf('./data/sim_%d',round(1000*T)),'tForDyn','yForDyn','xForDyn','model');
    end
    
    yForDyn_at_tid = @(t)interp1(tForDyn,yForDyn,t);
    
    yForDynKalman = yForDyn_at_tid(tKalman);
    yMeasSim         = yForDynKalman + randn(size(yForDynKalman))*chol(R); 
end
