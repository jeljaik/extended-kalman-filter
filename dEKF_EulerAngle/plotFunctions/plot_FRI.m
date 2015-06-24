function [] = plot_FRI(XUpt, PUpt,XPred,PPred,tK, idx,idEnd,col,figIn)
    pfri_expect =  zeros(length(tK),3);
    pfri_predict =  zeros(length(tK),3);
    [pfri_expect(:,1),pfri_expect(:,2)] = computeFRIFromState(XUpt,PUpt,tK,idx);
    [pfri_predict(:,1),pfri_predict(:,2)] = computeFRIFromState(XPred,PPred,tK,idx);
    
    figure;
    subplot(2,1,1);
    plot(tK,pfri_expect(:,1),'b','LineWidth',2.0); hold on;
    plot(tK,pfri_predict(:,1),'r','LineWidth',2.0);
    
    ylabel('Estimate FRI_z');
    xlabel('Time t(sec)');
    
    subplot(2,1,2);
    plot(tK,pfri_expect(:,2),'b','LineWidth',2.0); hold on;
    plot(tK,pfri_predict(:,2),'r','LineWidth',2.0);
    
    xlabel('Estimate FRI_y');
    ylabel('Time t(sec)');
    
%     subplot(3,1,3);
%     plot(tK,pfri_expect(:,3),'b','LineWidth',2.0); hold on;
%     plot(tK,pfri_predict(:,3),'r','LineWidth',2.0);
%     
%     xlabel('Estimate FRI_z');
%     ylabel('Time t(sec)');
      
%     figure(figIn);

    pfri_expect(1:idEnd,3) = 0.01*ones(size(pfri_expect(1:idEnd,1)));
%     plot(pfri_expect(:,1),pfri_expect(:,2),'Color',col);
%     hold on;
%     title('Motion of FRI');

    %hold on;
    %ellipse(squeeze(2*sqrt(ellipse(pfri_covariance(end,1,1)))),squeeze(2*sqrt(ellipse(pfri_covariance(end,2,2)))),0,pfri_expect(end,1),pfri_expect(end,2),'r');
    axis tight;
    hold on;

    ylabel('Estimate FRI_z (m)');
    xlabel('Time t(sec)');
    set(gca,'FontSize',12);



    addpath('./plotFunctions/foot_plotting/util');
    addpath('./plotFunctions/foot_plotting/graphics');



    [V8_R, F_R, ~] = read_plot_foot_mesh(0, 0);
    %should order the points in some way
    %Flip right foot to obtain the left one
    %V8_L = V8_R;
    %F_L = F_R;
    %V8_L(:,2) = -1 * V8_L(:,2);
    %now move each foot far from the origin of the plot
    %V8_R(:,2) = V8_R(:,2);% + feetDistanceFromCenter;
    %V8_L(:,2) = V8_L(:,2);% - feetDistanceFromCenter;

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
    trisurf(F_R, V8_R(:,1),V8_R(:,2),V8_R(:,3),'FaceColor',[242/255,67/255,43/255],'facealpha', 0.4, 'EdgeColor', 'none');
    hold on
    %plot(V8_R(:,1),V8_R(:,2),'k-');
    %trisurf(F_L, V8_L(:,1),V8_L(:,2),V8_L(:,3),'FaceColor',[232/255,57/255,23/255],'facealpha', 0.7, 'EdgeColor', 'none');
    axis equal;
    xlabel('z (m)');
    ylabel('y (m)');
    zlabel('x (m)');
    
    hold on;
    plot3(pfri_expect(idx:idEnd,1),pfri_expect(idx:idEnd,2),pfri_expect(idx:idEnd,3),'.-', 'LineWidth',2,'MarkerSize', 5, 'Color', 'b');
    axis tight;
    axis equal;
    set(gca,'FontSize',12);
    a = axis();
    axis([a(1:4)*1.1 a(5:6)]);

end
%a = 5;