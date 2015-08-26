function [model,transforms,leg_ft] = footComputations(inertial,leg_ft,expPath,leg_choice,model);

%We basically set the robot state from the state informtion available in
%the data log. Obtain coordinate transforms and link inertias, velocities and
%accelerations from  the automatically computed forward kinematics and
%perform a RNEA to express the l_leg_FT sensor measurement at the l_foot_FT
%sensor frame

dynComp = iDynTree.DynamicsComputations();

loadModel = dynComp.loadRobotModelFromFile('./model.urdf','urdf'); % Loading iCubGenova03 model.urdf
regCheck = dynComp.isValid(); %checking if regressor generator is correctly configured

if(strcmp(leg_choice,'left')== 1)
      footCheck = dynComp.setFloatingBase('l_foot'); %Setting floating base to the l_foot
      base_link = dynComp.getFloatingBase();
      idx = dynComp.getFrameIndex(base_link);
      Nb = dynComp.getFrameIndex('l_upper_leg');
      
else if(strcmp(leg_choice,'right')== 1)
      footCheck = dynComp.setFloatingBase('r_foot'); %Setting floating base to the l_foot
      base_link = dynComp.getFloatingBase();
      idx = dynComp.getFrameIndex(base_link);
      Nb = dynComp.getFrameIndex('r_upper_leg');
    end
end

dofInternal = dynComp.getNrOfDegreesOfFreedom; 

samplesToTrim = 50;


%% Obtaining joint position, velocities and accelerations
torso_state = importdata(strcat(expPath,'torso/state:o/data.log'));
torso.timeStamp = torso_state(:,2) - torso_state(1,2) ;
torso.q = torso_state(:,3:end).*(pi/180);
torso.dq = zeros(size(torso.timeStamp));
torso.ddq = zeros(size(torso.timeStamp));

right_leg_state = importdata(strcat(expPath,'right_leg/state:o/data.log'));
right_leg.timeStamp = right_leg_state(:,2) - right_leg_state(1,2) ;
right_leg.q = right_leg_state(:,3:end).*(pi/180);
right_leg.dq = zeros(size(right_leg.timeStamp));
right_leg.ddq = zeros(size(right_leg.timeStamp));


left_leg_state = importdata(strcat(expPath,'left_leg/state:o/data.log'));
left_leg.timeStamp = left_leg_state(:,2) - left_leg_state(1,2) ;
left_leg.q = left_leg_state(:,3:end).*(pi/180);
left_leg.dq = zeros(size(left_leg.timeStamp));
left_leg.ddq = zeros(size(left_leg.timeStamp));


torso = smoothAndEstimateVelAcc(torso);
right_leg = smoothAndEstimateVelAcc(right_leg);
left_leg = smoothAndEstimateVelAcc(left_leg);  %smoothAndEstimateVelAcc gives you the time derivatives dq and ddq

% torsoState = trimDataset(torsoSmooth,samplesToTrim)'

%% Obtaining angular acceleration from IMU angular velocity

load('IMUOffset.mat','com_R_imu');

omega.q = inertial.data(:,7:9);
omega.q = (com_R_imu*omega.q')';
omega.timeStamp = inertial.t;
omega.dq = zeros(size(omega.timeStamp));
omega.ddq = zeros(size(omega.timeStamp));
omega = smoothAndEstimateVelAcc(omega);


acc.q = inertial.data(:,4:6);
acc.q = (com_R_imu*acc.q')';
acc.timeStamp = inertial.t;
acc.dq = zeros(size(acc.timeStamp));
acc.ddq = zeros(size(acc.timeStamp));
acc = smoothAndEstimateVelAcc(acc);


torso = trimDataset(torso,samplesToTrim);
left_leg = trimDataset(left_leg,samplesToTrim);
right_leg = trimDataset(right_leg,samplesToTrim);
omega = trimDataset(omega,samplesToTrim);
acc = trimDataset(acc,samplesToTrim);

t = min([length(acc.timeStamp),length(omega.timeStamp),length(torso.timeStamp),length(right_leg.timeStamp),length(left_leg.timeStamp),length(leg_ft.t)]);

%% Initializing objects to set Robot state
q = iDynTree.VectorDynSize();
q_dot = iDynTree.VectorDynSize();
q_dotdot = iDynTree.VectorDynSize();
base_vel = iDynTree.Twist();
base_acc = iDynTree.ClassicalAcc();
world_T_base = iDynTree.Transform();
world_gravity = iDynTree.SpatialAcc(); %by default set to 0, because base acceleration includes gravity components
lin_vel = zeros(3,1);


f_foot = zeros(t,6);
f_leg =  zeros(t,6);


M = zeros(t,1);
Ifoot = dynComp.getLinkInertia(base_link);
mfoot = Ifoot.getMass(); % mass of the foot
Ic = Ifoot.getRotationalInertiaWrtCenterOfMass();

foot_p_B = Ifoot.getCenterOfMass();
B_p_foot = -foot_p_B.toMatlab();

mleg = 0;




for i = 1 : t

    mleg = mleg + mfoot; % cumulative mass - mass of other links will be added to express mass of leg
    
    %q, qdot and qdotdot are vectors of size with number of degrees of freedom
    jointpos = zeros(dofInternal,1);
    jointpos(1:6,1) = left_leg.q(i,1:6);
    jointpos(7:12,1) = right_leg.q(i,1:6);
    jointpos(15,1) = torso.q(i,1);
    jointpos(14,1) = torso.q(i,2);
    jointpos(13,1) = torso.q(i,3);
    q.fromMatlab(jointpos);    
       
    jointvel = zeros(dofInternal,1);
    jointvel(1:6,1) = left_leg.dq(i,1:6);
    jointvel(7:12,1) = right_leg.dq(i,1:6);
    jointvel(15,1) = torso.dq(i,1);
    jointvel(14,1) = torso.dq(i,2);
    jointvel(13,1) = torso.dq(i,3);
    q_dot.fromMatlab(jointvel);
        
    jointacc = zeros(dofInternal,1);
    jointacc(1:6,1) = left_leg.ddq(i,1:6);
    jointacc(7:12,1) = right_leg.ddq(i,1:6);
    jointacc(15,1) = torso.ddq(i,1);
    jointacc(14,1) = torso.ddq(i,2);
    jointacc(13,1) = torso.ddq(i,3);
    q_dotdot.fromMatlab(jointacc);
    
    ang_vel = omega.q(i,1:3)';
    base_vel.fromMatlab([lin_vel;ang_vel]);
        
    lin_acc = acc.q(i,1:3)';
    ang_acc = omega.dq(i,1:3)';
    base_acc.fromMatlab([lin_acc;ang_acc]);
    
    
    %Set robot state      
    state_check(i) = dynComp.setRobotState(q,q_dot,q_dotdot,world_T_base,base_vel,base_acc,world_gravity);
    if(state_check(i) == 1)
      
      footFT_X_legFT = dynComp.getRelativeTransform(idx,Nb); %coordinate transform between l_foot and l_hip_3
      X_FT = footFT_X_legFT.asAdjointTransformWrench();
      ft_leg = iDynTree.Wrench();
      ft_leg.fromMatlab([leg_ft.f(i,1:3)';leg_ft.mu(i,1:3)'])
      ft_leg_transformed = footFT_X_legFT*ft_leg;
      f_leg = ft_leg_transformed.toMatlab();
      f_foot(i,1:6) =  f_leg';  %to obtain f_leg from data log
      
      for j = idx - 1 : -1 : Nb  %RNEA
          footFT_X_Nb = dynComp.getRelativeTransform(idx,j);
          X_Nb = footFT_X_Nb.asAdjointTransformWrench();
          
          Il = dynComp.getLinkInertia(j);
          m = Il.getMass();
          I = Il.getRotationalInertiaWrtCenterOfMass();
          I = I.toMatlab();
          
          a = dynComp.getFrameProperSpatialAcceleration(j);
          a_link = a.toMatlab();
          v_dot = a_link(1:3);
          omega_dot = a_link(4:6);
          
          v = dynComp.getFrameTwist(j);
          v_link = v.toMatlab();
          v = v_link(1:3); 
          omega_b = v_link(4:6);
         
          h_dot = [m*eye(3)*v_dot + S(omega_b)*m*v; I*omega_dot + S(omega_b)*I*omega_b];
          hdot =  iDynTree.Wrench();
          hdot.fromMatlab(h_dot);
           
%           if(j > 3) %consider only upto upper leg - ignore l_hip_3 and _2
          mleg = mleg + m;
%           end
          f_int = footFT_X_Nb * hdot;
          f_foot(i,1:6) = f_foot(i,1:6) - f_int.toMatlab()'; %output gives a NAN value for the moments because of linkInertia of l_hip_3
      end
            M(i,1) = mleg;
            mleg = 0;
    end
        
    
end
model.m = mfoot;
model.I = Ic.toMatlab();
transforms.B_R_imu = com_R_imu;
transforms.B_adjT_foot = [eye(3) zeros(3);eye(3)*S(B_p_foot) eye(3)];
leg_ft.f = f_foot(:,1:3);
leg_ft.mu = f_foot(:,4:6);
leg_ft.t = 1:t;
