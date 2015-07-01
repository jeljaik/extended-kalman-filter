% Genova 3 Settembre 2014
% Author Francesco Nori
%
% This code creates plots for the CoDyCo first year demo. The robot is
% initially standing on the two feet and after some time instantiates a
% movement towards the edge of a table in front of him. At contat with the
% table the robot starts controlling the interaction force at the contact.


%% Load
% clear all %this clears also the breakpoints
clearvars

parts = {'leftArm'; 'rightArm'; 'torso'; 'rightLeg'; 'leftLeg'};
type  = {''; 'torques'};

folderPrefix = 'iCubGenova01CoDyCoDemoData/dumpWB/';

for i = 1:length(parts)
    for j = 1: length(type)
        folders = [parts{i} type{j}];
        data        = load([folderPrefix folders '/data.log']);
        eval(['time_' folders '= data(:,2);']);
        eval([folders '= data(:,3:end);']);
    end
end

parts = {'leftArm'; 'rightArm'; 'rightLeg'; 'leftLeg'; 'leftFoot'; 'rightFoot'};
type  = {'FTS'};

for i = 1:length(parts)
    for j = 1: length(type)
        folders = [parts{i} type{j}];
        data        = load([folderPrefix folders '/data.log']);
        eval(['time_' folders '= data(:,2);']);
        eval([folders '= data(:,3:end);']);
    end
end

%% Visualize
close all
time_leftArmFTS = time_leftArmFTS - time_leftArmFTS(1);
leftArmFTS(:,1) = leftArmFTS(:,1) - leftArmFTS(1,1);
leftArmFTS(:,2) = leftArmFTS(:,2) - leftArmFTS(1,2);
leftArmFTS(:,3) = leftArmFTS(:,3) - leftArmFTS(1,3);

plot(leftArmFTS(:,1:3));
% [XL,~] = ginput(2);
XL(1) = 9272;
XL(2) = 10488;

subIndex = ceil(XL(1)) : 10: floor(XL(2));
A = [zeros(1,length(subIndex)); zeros(1,length(subIndex)); linspace(0,1, length(subIndex))];
j = 1;
contactPositionLeft = [0.1, 0.1, 0.5];

h = figure;
for i = subIndex
    quiver3(contactPositionLeft(1), contactPositionLeft(2), contactPositionLeft(3),...
        0.01*leftArmFTS(i,1)', 0.01*leftArmFTS(i,2)', 0.01*leftArmFTS(i,3)', 'r', 'Color', A(:,j))
    j = j+1;
    hold on    
end
axis equal    

%right arm
figure
time_rightArmFTS = time_rightArmFTS - time_rightArmFTS(1);
rightArmFTS(:,1) = rightArmFTS(:,1) - rightArmFTS(1,1);
rightArmFTS(:,2) = rightArmFTS(:,2) - rightArmFTS(1,2);
rightArmFTS(:,3) = rightArmFTS(:,3) - rightArmFTS(1,3);

plot(rightArmFTS(:,1:3));
% [XR,~] = ginput(2);
XR(1) = 9420;
XR(2) = 10451;

subIndex = ceil(XR(1)) : 10: floor(XR(2));
A = [zeros(1,length(subIndex)); zeros(1,length(subIndex)); linspace(0,1, length(subIndex))];
j = 1;
contactPositionRight = [0.1, -0.1, 0.5];

figure(h);
for i = subIndex
    quiver3(contactPositionRight(1), contactPositionRight(2), contactPositionRight(3),...
        0.01*rightArmFTS(i,1)', 0.01*rightArmFTS(i,2)', 0.01*rightArmFTS(i,3)', 'r', 'Color', A(:,j))
    j = j+1;
    hold on    
end
axis equal

%remove data outside XL range
leftFootFTS(XL(2):end, :) = [];
leftFootFTS(1:XL(1), :) = [];
rightFootFTS(XL(2):end, :) = [];
rightFootFTS(1:XL(1), :) = [];

%draw CoPs on feet
drawCoPs(leftFootFTS, rightFootFTS, h, 0);

