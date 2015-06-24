function [] = drawCoP_and_CoM()
%%Draw feet
%First draw left and right foot
addpath(genpath('util'));
addpath(genpath('graphics'));
close all;

feetDistanceFromCenter = 0.07; %40 cm

[V8_R, F_R, ~] = read_plot_foot_mesh(1, 0);
%should order the points in some way
%Flip right foot to obtain the left one
V8_L = V8_R;
F_L = F_R;
V8_L(:,2) = -1 * V8_L(:,2);
%now move each foot far from the origin of the plot
V8_R(:,2) = V8_R(:,2) + feetDistanceFromCenter;
V8_L(:,2) = V8_L(:,2) - feetDistanceFromCenter;

%Prepare window
figure1 = figure('Color',...
    [0.972549021244049 0.972549021244049 0.972549021244049]);

% Create axes
axes('Parent',figure1,'ZTickLabel','','FontSize',16,...
    'DataAspectRatio',[1 1 1]);
% Create axes
% axes1 = axes('Parent',figure1,'ZTickLabel',{},'ZTick',zeros(1,0),...
%     'YGrid','on',...
%     'XGrid','on');
% hold(axes1,'all');

%Plot feet
trisurf(F_R, V8_R(:,1),V8_R(:,2),V8_R(:,3),'FaceColor',[232/255,57/255,23/255],'facealpha', 0.7, 'EdgeColor', 'none');
hold on
% plot(V8_R(:,1),V8_R(:,2),'k-');
trisurf(F_L, V8_L(:,1),V8_L(:,2),V8_L(:,3),'FaceColor',[232/255,57/255,23/255],'facealpha', 0.7, 'EdgeColor', 'none');
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');

%% Balancing left-right data
%Load data from CoDyCo Demo
demoData = load('Data/balancing_left_right_test1.mat');
xcomInitial = [0, demoData.xcom.Data(1,2), 0];  %offset only Y coordinate
firstSample = 4001;
time = demoData.xcom.Time;
xcomData = demoData.xcom.Data;
left_leg_wrench = demoData.left_leg_wrench.Data;
left_leg_wrench = sgolayfilt(left_leg_wrench, 3, 81);
right_leg_wrench = demoData.right_leg_wrench.Data;
right_leg_wrench = sgolayfilt(right_leg_wrench, 3, 81);

%speedup: remove all data before first usable sample
time(1:firstSample - 1,:) = [];
xcomData(1:firstSample - 1,:) = [];
xcomData(:,4:end) = [];
xcomData = xcomData - repmat(xcomInitial, size(xcomData, 1), 1); %CoM is w.r.t world in right foot. (I take the first CoM value as 0,0,0)
left_leg_wrench(1:firstSample - 1, :) = [];
left_leg_wrench(:, 7:8) = [];
right_leg_wrench(1:firstSample - 1, :) = [];
right_leg_wrench(:, 7:8) = [];

%%The reference frames of the wrenches taken from WBD3 are the same as the
%%end-effector frames they refer to.
%[0 0 1; 0 -1 0; 1 0 0];
T = [0  0 1;
     0 -1 0;
     1  0 0];
leftFootWrench = zeros(size(left_leg_wrench));
rightFootWrench = zeros(size(right_leg_wrench));
for i = 1 : size(left_leg_wrench, 1)
    leftFootWrench(i,:) = blkdiag(T,T) * left_leg_wrench(i,:)';
    rightFootWrench(i,:) = blkdiag(T,T) * right_leg_wrench(i,:)';
end

%CoP = [- mu_y / f_z ; + mu_x / f_z]
leftFootCOP = [-leftFootWrench(:,5) ./ leftFootWrench(:,3), leftFootWrench(:,4) ./ leftFootWrench(:,3)];
rightFootCOP = [-rightFootWrench(:,5) ./ rightFootWrench(:,3), rightFootWrench(:,4) ./ rightFootWrench(:,3)];

%translation along y of feetDistanceFromCenter
leftFootCOP(:,2) = leftFootCOP(:,2) - feetDistanceFromCenter;
rightFootCOP(:,2) = rightFootCOP(:,2) + feetDistanceFromCenter;

leftFootCoPAvg = mean(leftFootCOP);
rightFootCoPAvg = mean(rightFootCOP);
leftFootCoPStd = std(leftFootCOP);
rightFootCoPStd = std(rightFootCOP);

% disp('Final CoPs');
% disp('Left');
% leftFootCOP
% disp('Right');
% rightFootCOP

ZPlotCoordinate = 0.02;

samples = 3;
dataSize = size(leftFootCOP, 1);

fixedDataIndexes = 1;
if fixedDataIndexes == 1
    %take specific data
    finalValue = 4957;
    initialValue = 3702;

    batchSize = (finalValue - initialValue) / (samples - 1);
    leftFootCOPDownSampled = zeros(samples, 2);
    rightFootCOPDownSampled = zeros(samples, 2);
    comDownSampled = zeros(samples, 3);
    totalForces = zeros(samples, 2);

    leftFootCOPDownSampled(1,:) = leftFootCOP(initialValue,:);
    rightFootCOPDownSampled(1,:) = rightFootCOP(initialValue,:);
    comDownSampled(1,:) = xcomData(initialValue,:);
    totalForces(1,:) = leftFootWrench(initialValue,1:2) + rightFootWrench(initialValue,1:2);

    leftFootCOPDownSampled(end,:) = leftFootCOP(finalValue,:);
    rightFootCOPDownSampled(end,:) = rightFootCOP(finalValue,:);
    comDownSampled(end,:) = xcomData(finalValue,:);
    totalForces(end,:) = leftFootWrench(finalValue,1:2) + rightFootWrench(finalValue,1:2);

    for i = 2: samples - 1
        leftFootCOPDownSampled(i,:) = leftFootCOP(batchSize * i,:);
        rightFootCOPDownSampled(i,:) = rightFootCOP(batchSize * i,:);
        comDownSampled(i,:) = xcomData(batchSize * i,:);
        totalForces(i,:) = leftFootWrench(batchSize * i,1:2) + rightFootWrench(batchSize * i,1:2);
    end
else
    %simple downsampling
    leftFootCOPDownSampled = downsample(leftFootCOP, ceil(dataSize/samples));
    rightFootCOPDownSampled = downsample(rightFootCOP, ceil(dataSize/samples));
    comDownSampled = downsample(xcomData, ceil(dataSize/samples));
end

%tuned on the plot
col=hsv(samples);
scaleFactor = 1/180;
for i = 1 : samples
    x_coord = [comDownSampled(i,1), comDownSampled(i,1) + scaleFactor .* totalForces(i,1)];
    y_coord = [comDownSampled(i,2), comDownSampled(i,2) + scaleFactor .* totalForces(i,2)];
    z_coord = [comDownSampled(i,3), comDownSampled(i,3)];

    plot3(comDownSampled(i,1), comDownSampled(i,2), comDownSampled(i,3)', '.', 'MarkerSize', 25, 'Color', col(i, :));
    arrow3d(x_coord, y_coord, z_coord, .8, 0.001, 2 * 0.001, col(i, :));
end
plot3(comDownSampled(:,1), comDownSampled(:,2), comDownSampled(:,3)', 'b--', 'Marker', 'none');



% plot3(leftFootCOPDownSampled(:,1)', leftFootCOPDownSampled(:,2)', repmat(-0.02, samples, 1)', 'LineStyle', ':', 'Marker', '.', 'Color',[40/255, 53/255, 232/255], 'MarkerSize', 52);
% plot3(rightFootCOPDownSampled(:,1), rightFootCOPDownSampled(:,2), repmat(-0.02, samples, 1), '.', 'Color',[40/255, 53/255, 232/255], 'MarkerSize', 52);    

% plot(leftFootCOPDownSampled(:,1)', leftFootCOPDownSampled(:,2)', 'LineStyle', '--', 'Marker', '.', 'LineWidth', 1, 'Color',[40/255, 53/255, 232/255], 'MarkerSize', 10);
% plot(rightFootCOPDownSampled(:,1), rightFootCOPDownSampled(:,2), 'LineStyle', '--', 'Marker', '.', 'LineWidth', 1, 'Color',[40/255, 53/255, 232/255], 'MarkerSize', 10);    
plot3(leftFootCOPDownSampled(:,1)', leftFootCOPDownSampled(:,2)', repmat(ZPlotCoordinate, samples, 1)', 'LineStyle', '--', 'Marker', '.', 'LineWidth', 1, 'Color',[40/255, 53/255, 232/255], 'MarkerSize', 10);
plot3(rightFootCOPDownSampled(:,1), rightFootCOPDownSampled(:,2), repmat(ZPlotCoordinate, samples, 1)', 'LineStyle', '--', 'Marker', '.', 'LineWidth', 1, 'Color',[40/255, 53/255, 232/255], 'MarkerSize', 10);    

[leftMeanX, leftMeanY] = buildEllipse(leftFootCoPAvg(1), leftFootCoPAvg(2), ...
    leftFootCoPStd(1), leftFootCoPStd(2));
[rightMeanX, rightMeanY] = buildEllipse(rightFootCoPAvg(1), rightFootCoPAvg(2), ...
    rightFootCoPStd(1), rightFootCoPStd(2));

fill3(leftMeanX, leftMeanY, repmat(ZPlotCoordinate, size(leftMeanX), 1), [255/255, 194/255, 13/255], 'FaceColor',[255/255, 194/255, 13/255], 'EdgeColor',[255/255, 194/255, 13/255],'FaceAlpha', 0.5);
fill3(rightMeanX, rightMeanY,repmat(ZPlotCoordinate, size(leftMeanX), 1),[255/255, 194/255, 13/255], 'FaceColor',[255/255, 194/255, 13/255], 'EdgeColor',[255/255, 194/255, 13/255],'FaceAlpha', 0.5);
% plot(leftMeanX, leftMeanY, 'LineStyle', '-', 'Marker', 'none', 'LineWidth', 1, 'Color',[255/255, 194/255, 13/255]);
% plot(rightMeanX, rightMeanY, 'LineStyle', '-', 'Marker', 'none', 'LineWidth', 1, 'Color',[255/255, 194/255, 13/255]);


plot3(leftFootCoPAvg(1), leftFootCoPAvg(2), ZPlotCoordinate, 'LineStyle', '--', 'Marker', '.', 'LineWidth', 1, 'Color','k', 'MarkerSize', 15);
plot3(rightFootCoPAvg(1), rightFootCoPAvg(2), ZPlotCoordinate, 'LineStyle', '--', 'Marker', '.', 'LineWidth', 1, 'Color','k', 'MarkerSize', 15);    

% leftFootNorm = norm([leftFoot(1,3), leftFoot(1,4), 0]);
% rightFootNorm = norm([rightFoot(1,3), rightFoot(1,4), 0]);

% scaleFactor = 0.1;
% 
% arrow3d(scaleFactor .* [0, leftFoot(1,3)/leftFootNorm], ...
%     scaleFactor .* ([0, leftFoot(1,4)/leftFootNorm]) - feetDistanceFromCenter, ...
%     scaleFactor .* (-[0, 0]) ,.8, 0.005);
% 
% arrow3d(scaleFactor .* [0, rightFoot(1,3)/rightFootNorm], ...
%     scaleFactor .* ([0, rightFoot(1,4)/rightFootNorm]) + feetDistanceFromCenter, ...
%     scaleFactor .* (-[0, 0]) ,.8, 0.005);

axis auto
axis xy equal
set(gca,'ZTickLabel',[]);

% view(90, -90); %view XY plane

xlabel('x_{CoP}','VerticalAlignment','bottom','Rotation',90,...
    'HorizontalAlignment','center',...
    'FontSize',24);

% Create ylabel
ylabel('y_{CoP}','VerticalAlignment','cap','HorizontalAlignment','center',...
    'FontSize',24);

% axisScale = axis;
% axisScale(6) = 1.1;
% axis(axisScale);
% printImage(figure1, 'cop3.pdf');
% saveTightFigure(figure1, 'cop.pdf');



% This is 2d only
% arrowObj = annotation('arrow', [0.1 0.1], [0.5 0.5]);
% set(arrowObj, 'Units', 'centimeters');
% set(arrowObj, 'Position', [1 1 3 5]);

end

function [xc, yc] = buildEllipse(x,y, xRad, yRad)
ang=0:0.01:2*pi; 
xc = x + xRad * cos(ang);
yc = y + yRad * sin(ang);
end