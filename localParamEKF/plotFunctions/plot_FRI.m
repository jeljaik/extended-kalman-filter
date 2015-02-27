function [] = plot_FRI(XUpt, P,tK, idx,idEnd,col,figIn)

muo_x_B = XUpt(idx:end,13); muo_x_sigma = squeeze(2*sqrt(P(13,13,idx:end)))';
muo_y_B = XUpt(idx:end,14); muo_y_sigma = squeeze(2*sqrt(P(14,14,idx:end)))';
muo_z_B = XUpt(idx:end,15); muo_z_sigma = squeeze(2*sqrt(P(15,15,idx:end)))';

muc_x_B = XUpt(idx:end,19); muc_x_sigma = squeeze(2*sqrt(P(19,19,idx:end)))';
muc_y_B = XUpt(idx:end,20); muc_y_sigma = squeeze(2*sqrt(P(20,20,idx:end)))';
muc_z_B = XUpt(idx:end,21); muc_z_sigma = squeeze(2*sqrt(P(21,21,idx:end)))';

SK = S([0 0 0.18102]'); %transforming body frame to foot frame
%* foDash' + muoDash';

fc_x_B = XUpt(idx:end,10); fc_x_B_sigma = squeeze(2*sqrt(P(10,10,idx:end)))';
fc_y_B = XUpt(idx:end,11); fc_y_B_sigma = squeeze(2*sqrt(P(11,11,idx:end)))';
fc_z_B = XUpt(idx:end,12); fc_z_sigma = squeeze(2*sqrt(P(12,12,idx:end)))';

fo_x_B = XUpt(idx:end,7); fo_x_B_sigma = squeeze(2*sqrt(P(7,7,idx:end)))';
fo_y_B= XUpt(idx:end,8); fo_y_B_sigma = squeeze(2*sqrt(P(8,8,idx:end)))';
fo_z_B = XUpt(idx:end,9); fo_z_sigma = squeeze(2*sqrt(P(9,9,idx:end)))';


mu_o_B = [muo_x_B muo_y_B muo_z_B];
%mu_o_B_sigma = [muo_x_B_sigma muo_y_B_sigma muo_z_B_sigma]:

mu_c_B = [muc_x_B muc_y_B muc_z_B];
%mu_c_B_sigma = [muc_x_B_sigma;muc_y_B_sigma;muc_z_B_sigma]:

f_o_B = [fo_x_B fo_y_B fo_z_B];
%f_o_B_sigma = [fo_x_B_sigma;fo_y_B_sigma;fo_z_B_sigma];

f_c_B = [fc_x_B fc_y_B fc_z_B];
%f_c_B_sigma = [fc_x_B_sigma;fc_y_B_sigma;fc_z_B_sigma];

mu_o = SK*f_o_B' + mu_o_B';
mu_c = SK*f_c_B' + mu_c_B';

muo_x = mu_o(1,:)';
muo_y = mu_o(2,:)';

muc_x = mu_c(1,:)';
muc_y = mu_c(2,:)';

fo_z = fo_z_B;
fc_z = fc_z_B;
%muo = SK*fo_z' + muo_x_B;
%muc_x = SK*fo_z' + muo_x_B;

%pcop_expect = zeros(size(muo_x,1),2);pcop_covariance = zeros(size(muo_y,1),2,2);
pfri_expect = zeros(size(muo_x,1),3);pfri_covariance = zeros(size(muo_y,1),2,2);
w = zeros(size(muo_x,1));
h = zeros(size(muo_x,1));

for i = 1:size(muo_x,1)
    %[pcop_expect(i,:),pcop_covariance(i,:,:)] = computeCOP(muc_x(i),muc_y(i),fc_z(i),...
    %    muc_x_sigma(i),muc_y_sigma(i),fc_z_sigma(i)) ;
    [pfri_expect(i,1:2),pfri_covariance(i,:,:)] = computeFRI(muo_x(i),muo_y(i),fo_z(i),...
        muo_x_sigma(i),muo_y_sigma(i),fo_z_sigma(i));
    w(i) = squeeze(2*sqrt(pfri_covariance(i,1,1)));
    h(i) = squeeze(2*sqrt(pfri_covariance(i,2,2)));
end
feetDistanceFromCenter = 0.07;%0.07; %40 cm

%%% I SWAPPED FRI X AND Y BECAUSE SYMBOLIC_FRI_COP MIGHT HAVE A BUG (I.E
%%% NEEDS SWAPPING AROUND).


figure(figIn);
pfri_expect(1:idEnd,1) = pfri_expect(1:idEnd,1) + feetDistanceFromCenter +0.22;
pfri_expect(1:idEnd,2) = pfri_expect(1:idEnd,2);
pfri_expect(1:idEnd,3) = 0.01*ones(size(pfri_expect(1:idEnd,3)));
%plot(pcop_expect(:,1),pcop_expect(:,2),'r'); hold on;
plot(pfri_expect(1:idEnd,2),pfri_expect(1:idEnd,1),'Color',col);
hold on;
%legend('FRI');
title('Motion of FRI');

%hold on;
%ellipse(squeeze(2*sqrt(ellipse(pfri_covariance(end,1,1)))),squeeze(2*sqrt(ellipse(pfri_covariance(end,2,2)))),0,pfri_expect(end,1),pfri_expect(end,2),'r');
axis tight;
hold on;

xlabel('Position x(m)');
ylabel('Position y(m)');

% 
% figure(figIn+1);
% subplot(2,1,1);
% plot(tK(idx:idEnd),pfri_expect(idx:idEnd,1),'Color',col); hold on;
% axis tight;
% title('Exp FRI');
% subplot(2,1,2);
% plot(tK(idx:idEnd), pfri_expect(idx:idEnd,2),'Color',col); hold on;
% axis tight;
% hold on;
% % 
% 
% figure(figIn+2);
% subplot(2,1,1);
% plot(tK(idx:idEnd),w(idx:idEnd),'Color',col); hold on;
% title('Ellipse dimensions FRI undertainity');
% subplot(2,1,2);
% plot(tK(idx:idEnd), h(idx:idEnd),'Color',col); hold on;
% axis tight;
% hold on;

addpath('./plotFunctions/foot_plotting/util');
addpath('./plotFunctions/foot_plotting/graphics');



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
%plot(V8_R(:,1),V8_R(:,2),'k-');
%trisurf(F_L, V8_L(:,1),V8_L(:,2),V8_L(:,3),'FaceColor',[232/255,57/255,23/255],'facealpha', 0.7, 'EdgeColor', 'none');
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');

%% removing distance from centertofeetdistance
hold on;
plot3(pfri_expect(idx:idEnd,2),pfri_expect(idx:idEnd,1),pfri_expect(idx:idEnd,3),'.-', 'LineWidth',2,'MarkerSize', 5, 'Color', col);


a = 5;