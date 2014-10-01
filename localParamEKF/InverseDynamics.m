function [f_B1_t, mu_B1_t, f_B2_t, mu_B2_t, x0] = InverseDynamics(T, model, plots)
% This code computes  inverse dynamics for a rigid body system assuming some linear 
% velocity and a desired orientation trajectory parametrized in ZYZ 
% Euler angles (roll, pitch and yaw).
% Author: Naveen Kuppuswamy
% Istituto Italiano di Tecnologia
% Department of Robotics, Brain and Cognitive Sciences (RBCS)

%% Desired orientation trajectory
tc = 0:model.dtInvDyn:T;

%alpha   = pi/4+pi/8*cos(tc)'; 
%upsilon = pi/4+pi/8*ones(length(tc),1); 

%alpha   = pi/4*cos(tc/T)';  zeros(length(tc),1);%
%alpha   = pi/4 + (pi/8)*cos((4*pi*tc)/T)';

Phi_theta =(0.5*pi) /T; % singularities lie at angles 0 and pi

alpha = zeros(length(tc),1);
upsilon =  0.05+0.9*sin(Phi_theta*tc);%0.05+(Phi_theta*(tc)).^2;
psi = zeros(length(tc),1); 
%upsilon = pi/16*ones(length(tc),1);%zeros(length(tc),1); 

psi     = zeros(length(tc),1);
Phi     = [    alpha      upsilon'    psi     ];

dalpha = zeros(length(tc),1);
dupsilon = 0.9*Phi_theta*cos(Phi_theta*tc);%Phi_theta*ones(length(tc),1);
dpsi = zeros(length(tc),1);
dPhi = [ dalpha  dupsilon'  dpsi ];

%Assuming foot position traces an ellipse of (semi major axis a, semi minor
%axis b).
ell_a = 0.25;
ell_b = 0.001;
ell_th = ((pi/T) - ((0.5*pi/T)*tc)  )';
d_th = -(0.5*pi)/T;
%Pos = zeros(length(tc),3);
%Pos =  [ell_a*cos(ell_theta); zeros(length(tc),1) ;ell_b*sin(ell_theta)];

%Pos = [ell_a*cos(ell_th) ; zeros(length(tc),1); ell_b*sin(ell_th)];
dPos =  [-ell_a*d_th*sin(ell_th)     , zeros(length(tc),1) ,  ell_b*d_th*cos(ell_th)];
ddPos = [-ell_a*(d_th^2)*cos(ell_th), zeros(length(tc),1) , -ell_b*(d_th^2)*sin(ell_th)];


%dPos =  [-ell_a*d_th*sin(ell_th)  ,    zeros(length(tc),1) ,  ell_b*d_th*cos(ell_th)];
%ddPos = [-ell_a*d_th*d_th*cos(ell_th) , zeros(length(tc),1) , -ell_b*d_th*d_th*sin(ell_th)];



%% Required external wrenches
[f_B1_t, mu_B1_t, f_B2_t, mu_B2_t, omega_B] = rigidBodyInvDyn(Phi, dPhi, dPos, ddPos, tc, model, 0.5, 0.5);

%% Plots
if plots
    figure(1);
    subplot(2,1,1);
    plot(tc,f_B1_t(tc)); hold on;
    plot(tc,f_B2_t(tc)); axis tight; grid on;
    legend('{f^B}_1','{f^B}_2');
    xlabel('time t(sec)');
    ylabel('Force (N)');
    title('Inverse Dynamics');
    subplot(2,1,2);
    plot(tc,mu_B1_t(tc)); hold on;
    plot(tc,mu_B2_t(tc));axis tight; grid on;
    legend('{\mu ^B}_1','{\mu ^B}_2');
    xlabel('time t(sec)');
    ylabel('Wrench (Nm)');

    figure(2);
    subplot(3,2,1);
    plot(tc,dPos) ;hold on; axis tight;
    xlabel('time t(sec)');
    ylabel('Velocity (m/sec)');
    title('Desired motion');
    subplot(3,2,3);
    plot(tc,omega_B); hold on; axis tight;
    xlabel('time t(sec)');
    ylabel('Angular velocity (rad/sec)');

    subplot(3,2,5);
    plot(tc,ddPos); hold on; axis tight;
    xlabel('time t(sec)');
    ylabel('Acceleration (m/sec^2)');
end
%tc = t;%(1:end-1);


%Phi0 = Phi (1,:)';
%omega_B0 = omega_B(1,:)';
%dPos0 = dPos(1,:)';
%tc = tc(1:end-2); %% After numerical derivatives
x0= [dPos(1,:)';omega_B(1,:)';zeros(12,1);Phi(1,:)']; 
end