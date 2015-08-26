function [model,tMax,leg_ft,foot_ft,skin,inertial,transforms,expPath] = modelSensorParams_foot(whichLeg,whichSkin,numberOfExperiment,t_max,model,dataSet)
%% function returns the model parameters for treating the complete leg as a single rigid body.
%The function for now only returns the values corresponding to right leg


%% world gravity
model.B0_g = [0;0;9.8];
model.phi0 = [0; pi/2; 0];
B_R_G = euler2dcm(model.phi0);
model.G_g = -B_R_G'*model.B0_g;

%% Setting datasets
if(strcmp(dataSet,'old')== 1)
    %     Left Leg
    offsets.FT2 = [-4.11735 111.148 -131.104 -13.423 1.36924 -1.04304]; %old dataset
    %     Left Foot
    offsets.FT3 = [-52.0296 -5.64247 -18.0923 -0.516052 9.91345 0.717567]; %old dataset
    %     Right Leg
    offsets.FT4 = [-5.65084 -10.5031 -24.6174 1.95779 2.61084 0.0036913]; %old dataset
    %     Right Foot
    offsets.FT5 = [-17.4992 -0.910681 18.0613 -0.824831 3.32657 -0.252851]; %old dataset
    
    
    expPath     = ['./robotData/backwardTipping/dumperTippingSetup0' ...
        num2str(numberOfExperiment) '/icub/'];
    load('IMUOffset.mat','com_R_imu'); % Comment if calling footComputations()
else if(strcmp(dataSet,'new')== 1)
        load('IMUOffset_New.mat','com_R_imu');
        switch(numberOfExperiment)
            case 1
                expPath = ['./robotData/compliant-surface-dataset-foot/bwdTippingOnCompliantSurface/backwardTippingCompliant3006201501/dumper/icub/'];
                % Left leg
                offsets.FT2 = [113.821837 104.494539 -214.493500 -10.198060  1.850435 -0.482308]; %new dataset 1
                % Left foot
                offsets.FT3 = [-23.237569 -38.295595 332.422470 0.706069 -6.038355 -0.497729]; %new dataset 1
                % Right leg
                offsets.FT4 = [-51.867380 50.283953 -60.974108 -10.459150 4.045945 0.237656]; %new dataset 1
                % Right Foot
                offsets.FT5 = [19.771900 22.325223 -117.840856 0.996938 4.261731 -0.213692]; %new dataset 1
            case 2
                
                expPath = ['./robotData/compliant-surface-dataset-foot/bwdTippingOnCompliantSurface/backwardTippingCompliant3006201502/dumper/icub/'];
                
                % Left leg
                offsets.FT2 = [104.548459 111.461942 -219.925026 -13.095005 -0.556614 -0.001912]; %new dataset 2
                % Left Foot
                offsets.FT3 = [-27.621335 -31.330527 327.876023 0.961083 -6.022130 0.058773]; %new dataset 2
                % Right Leg
                offsets.FT4 = [-46.677968 36.227873 -61.667356 -8.496646 4.516830 0.671148]; %new dataset 2
                % Right foot
                offsets.FT5 = [28.027292 14.668884 -116.054800 -0.516083 2.174395 0.130253]; %new dataset 2
            case 3
                expPath = ['./robotData/compliant-surface-dataset-foot/bwdTippingOnCompliantSurface/backwardTippingCompliant3006201503/dumper/icub/'];
                
                % Left leg
                offsets.FT2 = [105.398590 109.929423 -221.307414 -12.347826 -0.434969 -0.191445]; %new dataset 3
                % Left Foot
                offsets.FT3 = [-24.483119 -32.753256 325.118028 0.851408 -6.514582 -0.146724]; %new dataset 3
                % Right Leg
                offsets.FT4 = [-48.811644 40.813063 -55.637579 -9.656452 2.872986 0.551312]; %new dataset 3
                % Right foot
                offsets.FT5 = [28.219465 19.492839 -113.029109 -0.767924 1.462215 0.036121]; %new dataset 3
                
            case 4
                expPath = ['./robotData/compliant-surface-dataset-foot/bwdTippingOnCompliantSurface/backwardTippingCompliant3006201504/dumper/icub/'];
                % Left leg
                offsets.FT2 = [108.374177 116.559645 -220.709141 -14.103646 1.691940 -0.285750]; %new dataset 4
                % Left Foot
                offsets.FT3 = [-22.983853 -27.517107 320.436696 0.966418 -5.178292 -0.222516]; %new dataset 4
                % Right Leg
                offsets.FT4 = [-53.350973 36.368447 -49.121294 -8.398413 0.871890 0.178244]; %new dataset 4
                % Right foot
                offsets.FT5 = [24.752090 17.166970 -108.177663 -1.080416 1.323988 -0.388058]; %new dataset 4
        end
        
    end
end

%% OFFSETS.

left_leg_ft_offset = offsets.FT2;
right_leg_ft_offset = offsets.FT4;
left_foot_ft_offset = offsets.FT3;
right_foot_ft_offset = offsets.FT5;

leg_choice  = whichLeg;
skin_choice = whichSkin;

% Leg F/T analog sensor
% leg_ft_data   = importdata(strcat(expPath,leg_choice,'_leg/analog:o/data.log'));
% Foot F/T analog sensor
foot_ft_data  = importdata(strcat(expPath,leg_choice,'_foot/analog:o/data.log'));
% Foot skin data
skin_data     = importdata(strcat(expPath,'skin/',skin_choice,'_foot/data.log'));
% Inertial sensor attached to the foot (right)
inertial_data = importdata(strcat(expPath,'inertial/data.log'));


if(strcmp(leg_choice,'left')==1)
    leg_ft_offset = left_leg_ft_offset;
    foot_ft_offset = left_foot_ft_offset;
else
    leg_ft_offset = right_leg_ft_offset;
    foot_ft_offset = right_foot_ft_offset;
end

% uncomment this section and use footComptutations function is using old dataset
% leg_ft.t = leg_ft_data(:,2)-leg_ft_data(1,2);
% leg_ft.idx = leg_ft_data(:,1) - leg_ft_data(1,1);
% leg_ft_data(:,3:8) = leg_ft_data(:,3:8) - repmat(leg_ft_offset,size(leg_ft_data,1),1);
% leg_ft.f = leg_ft_data(:,3:5);
% leg_ft.mu = leg_ft_data(:,6:8);

%%
foot_ft.t = foot_ft_data(:,2)-foot_ft_data(1,2);
foot_ft.idx = foot_ft_data(:,1) - foot_ft_data(1,1);
foot_ft_data(:,3:8) = foot_ft_data(:,3:8) - repmat(foot_ft_offset,size(foot_ft_data,1),1);
foot_ft.f = foot_ft_data(:,3:5);
foot_ft.mu = foot_ft_data(:,6:8);

skin.t = skin_data(:,2) - skin_data(1,2);
skin.idx = skin_data(:,1) - skin_data(1,1);
skin.data = skin_data(:,3:end);

inertial.t = inertial_data(:,2)-inertial_data(1,2);
inertial.idx = inertial_data(:,1) - inertial_data(1,1);
inertial.data = inertial_data(:,3:end);


% [model,transforms,leg_ft] = footComputations(inertial,leg_ft,expPath,leg_choice,model);
%% Comment this section if calling footComputations() function
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
Ifoot = dynComp.getLinkInertia(base_link);
mfoot = Ifoot.getMass(); % mass of the foot
Ic = Ifoot.getRotationalInertiaWrtCenterOfMass();

foot_p_B = Ifoot.getCenterOfMass();
B_p_foot = -foot_p_B.toMatlab();
model.m = mfoot;
model.I = Ic.toMatlab();

transforms.B_R_imu = com_R_imu;
transforms.B_adjT_foot = [eye(3) zeros(3);eye(3)*S(B_p_foot) eye(3)];

%%
tMax = min([foot_ft.t(end),skin.t(end),inertial.t(end),t_max]);
leg_ft = struct();
% tMax = min([leg_ft.t(end),foot_ft.t(end),skin.t(end),inertial.t(end),t_max]);



end