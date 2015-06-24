function [] = drawCOPScript()
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

%Load data from CoDyCo Demo
leftFoot = load('Data/leftFootFTS/data.log');
rightFoot = load('Data/rightFootFTS/data.log');

firstSample = 1700;
%speedup: remove all data before first usable sample
leftFoot(1:firstSample - 1,:) = [];
rightFoot(1:firstSample - 1,:) = [];
%size the two vectors
minSize = min(size(leftFoot, 1), size(rightFoot, 1));
leftFoot(minSize:end,:) = [];
rightFoot(minSize:end,:) = [];

%remove offset (?)
%now hardcoded
%they are transformed to the reference frame of the sensor
%left leg (116.128128 0.116392 10.806414) (0.084782 0.386993 -0.817061)
fx_left_offset = 10.806414 - leftFoot(1,3);
fy_left_offset = 0.116392 - leftFoot(1,4);
fz_left_offset = -116.128128 - leftFoot(1,5);
mx_left_offset = -0.817061 - leftFoot(1,6);
my_left_offset = 0.386993 - leftFoot(1,7);
mz_left_offset = -0.084782 - leftFoot(1,8);
%right leg (115.21056 -2.091521 15.853994) (0.238595 0.770593 -0.809924)
fx_right_offset = 15.853994 - rightFoot(1,3);
fy_right_offset = -2.091521 - rightFoot(1,4);
fz_right_offset = -115.21056 - rightFoot(1,5);
mx_right_offset = -0.809924 - rightFoot(1,6);
my_right_offset = 0.770593 - rightFoot(1,7);
mz_right_offset = -0.238595 - rightFoot(1,8);

%update data
leftFoot(:,3) = leftFoot(:,3) + fx_left_offset;
leftFoot(:,4) = leftFoot(:,4) + fy_left_offset;
leftFoot(:,5) = leftFoot(:,5) + fz_left_offset;
leftFoot(:,6) = leftFoot(:,6) + mx_left_offset;
leftFoot(:,7) = leftFoot(:,7) + my_left_offset;
leftFoot(:,8) = leftFoot(:,8) + mz_left_offset;

rightFoot(:,3) = rightFoot(:,3) + fx_right_offset;
rightFoot(:,4) = rightFoot(:,4) + fy_right_offset;
rightFoot(:,5) = rightFoot(:,5) + fz_right_offset;
rightFoot(:,6) = rightFoot(:,6) + mx_right_offset;
rightFoot(:,7) = rightFoot(:,7) + my_right_offset;
rightFoot(:,8) = rightFoot(:,8) + mz_right_offset;

%CoP = [- mu_y / f_z ; + mu_x / f_z]
leftFootCOP = [-leftFoot(:,7) ./ leftFoot(:,5), leftFoot(:,6) ./ leftFoot(:,5)];
rightFootCOP = [-rightFoot(:,7) ./ rightFoot(:,5), rightFoot(:,6) ./ rightFoot(:,5)];

% disp('Initial CoPs');
% disp('Left');
% leftFootCOP
% disp('Right');
% rightFootCOP

%this are w.r.t. sensor frame
%transform to "world"
%rotation: just invert y coordinate
%translation along y of feetDistanceFromCenter
leftFootCOP(:,2) = -leftFootCOP(:,2) - feetDistanceFromCenter;
rightFootCOP(:,2) = -rightFootCOP(:,2) + feetDistanceFromCenter;

leftFootCoPAvg = mean(leftFootCOP);
rightFootCoPAvg = mean(rightFootCOP);
leftFootCoPStd = std(leftFootCOP);
rightFootCoPStd = std(rightFootCOP);

% disp('Final CoPs');
% disp('Left');
% leftFootCOP
% disp('Right');
% rightFootCOP



samples = 5;
leftFootCOPDownSampled = downsample(leftFootCOP, ceil(minSize/samples));
rightFootCOPDownSampled = downsample(rightFootCOP, ceil(minSize/samples));

% plot3(leftFootCOPDownSampled(:,1)', leftFootCOPDownSampled(:,2)', repmat(-0.02, samples, 1)', 'LineStyle', ':', 'Marker', '.', 'Color',[40/255, 53/255, 232/255], 'MarkerSize', 52);
% plot3(rightFootCOPDownSampled(:,1), rightFootCOPDownSampled(:,2), repmat(-0.02, samples, 1), '.', 'Color',[40/255, 53/255, 232/255], 'MarkerSize', 52);    

plot(leftFootCOPDownSampled(:,1)', leftFootCOPDownSampled(:,2)', 'LineStyle', '--', 'Marker', '.', 'LineWidth', 1, 'Color',[40/255, 53/255, 232/255], 'MarkerSize', 10);
plot(rightFootCOPDownSampled(:,1), rightFootCOPDownSampled(:,2), 'LineStyle', '--', 'Marker', '.', 'LineWidth', 1, 'Color',[40/255, 53/255, 232/255], 'MarkerSize', 10);    

[leftMeanX, leftMeanY] = buildEllipse(leftFootCoPAvg(1), leftFootCoPAvg(2), ...
    leftFootCoPStd(1), leftFootCoPStd(2));
[rightMeanX, rightMeanY] = buildEllipse(rightFootCoPAvg(1), rightFootCoPAvg(2), ...
    rightFootCoPStd(1), rightFootCoPStd(2));

fill(leftMeanX, leftMeanY,[255/255, 194/255, 13/255], 'FaceColor',[255/255, 194/255, 13/255], 'EdgeColor',[255/255, 194/255, 13/255],'FaceAlpha', 0.5);
fill(rightMeanX, rightMeanY,[255/255, 194/255, 13/255], 'FaceColor',[255/255, 194/255, 13/255], 'EdgeColor',[255/255, 194/255, 13/255],'FaceAlpha', 0.5);
% plot(leftMeanX, leftMeanY, 'LineStyle', '-', 'Marker', 'none', 'LineWidth', 1, 'Color',[255/255, 194/255, 13/255]);
% plot(rightMeanX, rightMeanY, 'LineStyle', '-', 'Marker', 'none', 'LineWidth', 1, 'Color',[255/255, 194/255, 13/255]);


plot(leftFootCoPAvg(1), leftFootCoPAvg(2), 'LineStyle', '--', 'Marker', '.', 'LineWidth', 1, 'Color','k', 'MarkerSize', 15);
plot(rightFootCoPAvg(1), rightFootCoPAvg(2), 'LineStyle', '--', 'Marker', '.', 'LineWidth', 1, 'Color','k', 'MarkerSize', 15);    

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

view(90, -90); %view XY plane

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

