% Inputs are in quaternion.

function plot_predictions(time, MM, PP, realOrientation)
    figure;
    % After transforming this series of quaternions into a 3D matrix of
    % doubles as done next ...
    MM_double = MM.double;
    % ... We need to rearrange it into matrix form, where each column
    % corresponds to a time series element.
    MM_double = permute(MM_double,[1 3 2]);
    realOrientation_double = realOrientation.double;
    realOrientation_double = permute(realOrientation_double, [1 3 2]);
    color1 = [0 67 88]/255;
    color2 = [31 138 112]/255;
    color3 = [190 219 57]/255;
    color4 = [253 116 0]/255;
    hold on;
    plot(time, realOrientation_double(1,:),'Color', color1', 'LineWidth',2);
    plot(time, realOrientation_double(2,:),'Color', color2', 'LineWidth',2);
    plot(time, realOrientation_double(3,:),'Color', color3', 'LineWidth',2);
    plot(time, realOrientation_double(4,:),'Color', color4', 'LineWidth',2); 
    
    plot(time, MM_double(1,:), 'Color', color1, 'LineWidth',2, 'LineStyle', ':');
    plot(time, MM_double(2,:), 'Color', color2, 'LineWidth',2, 'LineStyle', ':');
    plot(time, MM_double(3,:), 'Color', color3, 'LineWidth',2, 'LineStyle', ':');
    plot(time, MM_double(4,:), 'Color', color4, 'LineWidth',2, 'LineStyle', ':'); 
    
    axis tight;
    title('Comparing Quaternions');
    legend('realQuat1', 'realQuat2', 'realQuat3', 'realQuat4','estQuat1', 'estQuat2', 'estQuat3', 'estQuat4');
    
    figure; hold on;
    realOrientation_euler = realOrientation.EulerAngles('xyz');
    realOrientation_euler = permute(realOrientation_euler, [1 3 2]);
    
    MM_euler = MM.EulerAngles('xyz');
    MM_euler = permute(MM_euler, [1 3 2]);
    
    % Plotting estimates
    plot(time, realOrientation_euler(1,:),'Color', color1', 'LineWidth',2);
    plot(time, realOrientation_euler(2,:),'Color', color2', 'LineWidth',2);
    plot(time, realOrientation_euler(3,:),'Color', color3', 'LineWidth',2);
    
    plot(time, MM_euler(1,:), 'Color', color1, 'LineWidth',2, 'LineStyle', ':');
    plot(time, MM_euler(2,:), 'Color', color2, 'LineWidth',2, 'LineStyle', ':');
    plot(time, MM_euler(3,:), 'Color', color3, 'LineWidth',2, 'LineStyle', ':');
    
    % Plotting covariances
    shadedErrorBar(time, MM_euler(1,:), squeeze(2*sqrt(PP(1,1,:)))', {'Color', color1}, 1);
    shadedErrorBar(time, MM_euler(2,:), squeeze(2*sqrt(PP(2,2,:)))', {'Color', color2}, 1);
    shadedErrorBar(time, MM_euler(3,:), squeeze(2*sqrt(PP(3,3,:)))', {'Color', color3}, 1);
    axis tight;
    title('Comparing Euler Angles');
    legend('realEuler1', 'realEuler2', 'realEuler3','estEuler1', 'estEuler2', 'estEuler3');

end