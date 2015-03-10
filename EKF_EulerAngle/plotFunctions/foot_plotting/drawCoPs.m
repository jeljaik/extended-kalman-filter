function [] = drawCoPs(leftFootFTS, rightFootFTS, figureHandle, FTDataOffset)
%First draw left and right foot
addpath(genpath('util'));
addpath(genpath('graphics'));

feetDistanceFromCenter = 0.07; %this can be hardcoded

[V8_R, F_R, ~] = read_plot_foot_mesh(1, 0);
%should order the points in some way
%Flip right foot to obtain the left one
V8_L = V8_R;
F_L = F_R;
V8_L(:,2) = -1 * V8_L(:,2);
%now move each foot far from the origin of the plot
V8_R(:,2) = V8_R(:,2) + feetDistanceFromCenter;
V8_L(:,2) = V8_L(:,2) - feetDistanceFromCenter;

figure(figureHandle);

%Plot feet
trisurf(F_R, V8_R(:,1),V8_R(:,2),V8_R(:,3),'FaceColor',[232/255,57/255,23/255],'facealpha', 0.7, 'EdgeColor', 'none');
hold on
% plot(V8_R(:,1),V8_R(:,2),'k-');
trisurf(F_L, V8_L(:,1),V8_L(:,2),V8_L(:,3),'FaceColor',[232/255,57/255,23/255],'facealpha', 0.7, 'EdgeColor', 'none');
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');


%remove offset (?)
%now hardcoded
%they are transformed to the reference frame of the sensor
%left leg (116.128128 0.116392 10.806414) (0.084782 0.386993 -0.817061)
fx_left_offset = 10.806414 - leftFootFTS(1, FTDataOffset + 1);
fy_left_offset = 0.116392 - leftFootFTS(1, FTDataOffset + 2);
fz_left_offset = -116.128128 - leftFootFTS(1, FTDataOffset + 3);
mx_left_offset = -0.817061 - leftFootFTS(1, FTDataOffset + 4);
my_left_offset = 0.386993 - leftFootFTS(1, FTDataOffset + 5);
mz_left_offset = -0.084782 - leftFootFTS(1, FTDataOffset + 6);
%right leg (115.21056 -2.091521 15.853994) (0.238595 0.770593 -0.809924)
fx_right_offset = 15.853994 - rightFootFTS(1, FTDataOffset + 1);
fy_right_offset = -2.091521 - rightFootFTS(1, FTDataOffset + 2);
fz_right_offset = -115.21056 - rightFootFTS(1, FTDataOffset + 3);
mx_right_offset = -0.809924 - rightFootFTS(1, FTDataOffset + 4);
my_right_offset = 0.770593 - rightFootFTS(1, FTDataOffset + 5);
mz_right_offset = -0.238595 - rightFootFTS(1, FTDataOffset + 6);

%update data
leftFootFTS(:,FTDataOffset + 1) = leftFootFTS(:,FTDataOffset + 1) + fx_left_offset;
leftFootFTS(:,FTDataOffset + 2) = leftFootFTS(:,FTDataOffset + 2) + fy_left_offset;
leftFootFTS(:,FTDataOffset + 3) = leftFootFTS(:,FTDataOffset + 3) + fz_left_offset;
leftFootFTS(:,FTDataOffset + 4) = leftFootFTS(:,FTDataOffset + 4) + mx_left_offset;
leftFootFTS(:,FTDataOffset + 5) = leftFootFTS(:,FTDataOffset + 5) + my_left_offset;
leftFootFTS(:,FTDataOffset + 6) = leftFootFTS(:,FTDataOffset + 6) + mz_left_offset;

rightFootFTS(:, FTDataOffset + 1) = rightFootFTS(:, FTDataOffset + 1) + fx_right_offset;
rightFootFTS(:, FTDataOffset + 2) = rightFootFTS(:, FTDataOffset + 2) + fy_right_offset;
rightFootFTS(:, FTDataOffset + 3) = rightFootFTS(:, FTDataOffset + 3) + fz_right_offset;
rightFootFTS(:, FTDataOffset + 4) = rightFootFTS(:, FTDataOffset + 4) + mx_right_offset;
rightFootFTS(:, FTDataOffset + 5) = rightFootFTS(:, FTDataOffset + 5) + my_right_offset;
rightFootFTS(:, FTDataOffset + 6) = rightFootFTS(:, FTDataOffset + 6) + mz_right_offset;

%CoP = [- mu_y / f_z ; + mu_x / f_z]
leftFootCOP = [-leftFootFTS(:, FTDataOffset + 5) ./ leftFootFTS(:, FTDataOffset + 3), leftFootFTS(:, FTDataOffset + 4) ./ leftFootFTS(:, FTDataOffset + 3)];
rightFootCOP = [-rightFootFTS(:, FTDataOffset + 5) ./ rightFootFTS(:, FTDataOffset + 3), rightFootFTS(:, FTDataOffset + 4) ./ rightFootFTS(:, FTDataOffset + 3)];

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

minSize = min(size(leftFootFTS, 1), size(rightFootFTS, 1));
samples = 5;
leftFootCOPDownSampled = downsample(leftFootCOP, ceil(minSize/samples));
rightFootCOPDownSampled = downsample(rightFootCOP, ceil(minSize/samples));

figure(figureHandle);
heightOf2DPlot = 0.02;

plot3(leftFootCOPDownSampled(:,1)', leftFootCOPDownSampled(:,2)', repmat(heightOf2DPlot, samples, 1)', 'LineStyle', '--', 'Marker', '.', 'LineWidth', 1, 'Color',[40/255, 53/255, 232/255], 'MarkerSize', 10);
plot3(rightFootCOPDownSampled(:,1), rightFootCOPDownSampled(:,2), repmat(heightOf2DPlot, samples, 1)', 'LineStyle', '--', 'Marker', '.', 'LineWidth', 1, 'Color',[40/255, 53/255, 232/255], 'MarkerSize', 10);    

[leftMeanX, leftMeanY] = buildEllipse(leftFootCoPAvg(1), leftFootCoPAvg(2), ...
    leftFootCoPStd(1), leftFootCoPStd(2));
[rightMeanX, rightMeanY] = buildEllipse(rightFootCoPAvg(1), rightFootCoPAvg(2), ...
    rightFootCoPStd(1), rightFootCoPStd(2));

fill3(leftMeanX, leftMeanY, repmat(heightOf2DPlot, size(leftMeanX)), [255/255, 194/255, 13/255], 'FaceColor',[255/255, 194/255, 13/255], 'EdgeColor',[255/255, 194/255, 13/255],'FaceAlpha', 0.5);
fill3(rightMeanX, rightMeanY, repmat(heightOf2DPlot, size(leftMeanX)), [255/255, 194/255, 13/255], 'FaceColor',[255/255, 194/255, 13/255], 'EdgeColor',[255/255, 194/255, 13/255],'FaceAlpha', 0.5);

plot3(leftFootCoPAvg(1), leftFootCoPAvg(2), heightOf2DPlot, 'LineStyle', '--', 'Marker', '.', 'LineWidth', 1, 'Color','k', 'MarkerSize', 15);
plot3(rightFootCoPAvg(1), rightFootCoPAvg(2), heightOf2DPlot, 'LineStyle', '--', 'Marker', '.', 'LineWidth', 1, 'Color','k', 'MarkerSize', 15);    

axis auto
axis xy equal

end

function [xc, yc] = buildEllipse(x,y, xRad, yRad)
ang=0:0.01:2*pi; 
xc = x + xRad * cos(ang);
yc = y + yRad * sin(ang);
end

