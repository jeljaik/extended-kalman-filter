%% Bearings Only Tracking (BOT) demonstration with GHKF
%
%  Description:
%    In this example the Gauss-Hermite Kalman filter and Gauss-Hermite 
%    Rauch-Tung-Striebel smoother are used to estimate the position and 
%    velocity of a moving object on a plane. Two sensors track the position
%    of the object by returning noisy measurements of angular direction of  
%    the target. 
%
%  References:
%    Refer to the Toolbox documentation for details on the model.
%
%  See also:
%    ghkf_predict, ghkf_update, ghrts_smooth
%
%  Author:
%    Copyright (C) 2002, 2003 Simo Särkkä
%                  2007       Jouni Hartikainen
%                  2010       Arno Solin
%
%  Licence:
%    This software is distributed under the GNU General Public 
%    Licence (version 2 or later); please refer to the file 
%    Licence.txt, included with the software, for details.

%% Run the model

  keep_trajectory = 0;
  silent = 0;
  
  % Number of steps to advance at a time in animations. 
  steps = 2;
  
  % Handle to measurement model function
  h_func = @bot_h;

  % Create a bit curved trajectory and angle
  % measurements from two sensors
  S1 = [-1;-2];
  S2 = [1;1];
  sd = 0.05;
  dt = 0.01;
  if ~keep_trajectory
    % Accelerations for the object.
    a = zeros(1,500);
    a(1,50:100)  = pi/2/51/dt + 0.01*randn(1,51);
    a(1,200:250) = pi/2/51/dt + 0.01*randn(1,51);
    a(1,350:400) = pi/2/51/dt + 0.01*randn(1,51);
    x = [0;0;1;0];
    t = 0;
    X = [];
    Y = [];
    T = [];
    for i=1:500
      F = [0 0  1    0;...
           0 0  0    1;...
           0 0  0   a(i);...
           0 0 -a(i) 0];
      x = expm(F*dt)*x;
      y1 = atan2(x(2)-S1(2), x(1)-S1(1)) + sd * randn;
      y2 = atan2(x(2)-S2(2), x(1)-S2(1)) + sd * randn;
      t  = t + dt;
      X = [X x];
      T = [T t];
      Y = [Y [y1;y2]];
    end
  end

  %
  % Initialize GHKF to values
  %
  %   x = 0
  %   y = 0,
  %   dx/dt = 0
  %   dy/dt = 0
  %
  % with great uncertainty in velocity
  %
  M = [0;0;0;0];
  P = diag([0.1 0.1 10 10]);
  R = sd^2;
  qx = 0.1;
  qy = 0.1;
  F = [0 0 1 0;
       0 0 0 1;
       0 0 0 0;
       0 0 0 0];
  [A,Q] = lti_disc(F,[],diag([0 0 qx qy]),dt);

  clc;  clf;
  disp(['In this demonstration we track a moving object with two sensors, ',...
        'which gives only bearings of the object with respect to sensors position. ',...
       'The state of the system is estimated with GHKF.'])
  disp(' ');
  fprintf('Running GHKF...')
  %
  % Track and animate
  %
  MM = zeros(size(M,1),size(Y,2));
  PP = zeros(size(M,1),size(M,1),size(Y,2));
  ME = zeros(size(M,1),1);
  for k=1:size(Y,2)
    % Track with GHKF
    [M,P] = ghkf_predict(M,P,A,Q,[],3);
    [M,P] = ghkf_update(M,P,Y(:,k),h_func,R*eye(2),[S1 S2],3);
    MM(:,k)   = M;
    PP(:,:,k) = P;
    ME(k) = P(1,1) + P(2,2);
  end
  ghkf_rmse = sqrt(mean((X(1,:)-MM(1,:)).^2+(X(2,:)-MM(2,:)).^2));

  fprintf('Done!\n')
  disp(' ');
  disp(['The filtering results are now displayed sequentially. ',...
       'Notice how the estimates gets more accurate when the filter gets on the right track. ',...
       'The green ellipse around the current estimate (little blue circle) reflects the filters ',...
       'confidence intervals of the position estimate.']);
  disp(' ');
  disp('<push any key to proceed>');

  % Plot the filtered estimates sequentially
  if ~silent
      M = MM(:,1);  
      P = PP(:,:,1);
      EST = M;
      tt = (0:0.01:1)*2*pi;
      cc = repmat(M(1:2),1,length(tt)) + ...
        	  2*chol(P(1:2,1:2))'*[cos(tt);sin(tt)];
      h = plot(X(1,:),X(2,:),'r-',...
               M(1),M(2),'bo',...
               EST(1,:),EST(2,:),'b--',...
               cc(1,:),cc(2,:),'g-',...
               S1(1),S1(2),'k--',...
               S1(1),S1(2),'k^',...
               S2(1),S2(2),'k--',...
               S2(1),S2(2),'k^');
      legend('Location','NorthWest','Real trajectory','Current estimate','Estimated trajectory',...
             'Confidence interval','Measurements from sensors','Positions of the sensors');
      title('Bearings Only Tracking with GHKF.')
      axis([-1.5 1.5 -2.5 1.5]);
      
      EST = [];
      for k=1:steps:size(Y,2)
        M = MM(:,k);
        P = PP(:,:,k);
        EST = MM(:,1:k);

        % Confidence ellipse
        cc = repmat(M(1:2),1,length(tt)) + ...
	     2*chol(P(1:2,1:2))'*[cos(tt);sin(tt)];

        % Measurement directions
        len = 2.5;
        dx1 = len*cos(Y(1,k));
        dy1 = len*sin(Y(1,k));
        dx2 = len*cos(Y(2,k));
        dy2 = len*sin(Y(2,k));

        % Update graphics
        set(h(2),'xdata',M(1)); set(h(2),'ydata',M(2));
        set(h(3),'xdata',EST(1,:)); set(h(3),'ydata',EST(2,:)); 
        set(h(4),'xdata',cc(1,:)); set(h(4),'ydata',cc(2,:)); 
        set(h(5),'xdata',[S1(1);S1(1)+dx1]); set(h(5),'ydata',[S1(2);S1(2)+dy1]); 
        set(h(7),'xdata',[S2(1);S2(1)+dx2]); set(h(7),'ydata',[S2(2);S2(2)+dy2]); 
      pause
    end
  end
  
  clc;
  disp(['In this demonstration we track a moving object with two sensors, ',...
        'which gives only bearings of the object with respect to sensors position. ',...
       'The state of the system is estimated with GHKF.'])
  disp(' ');
  fprintf('Running GHKF...Done!\n')
  disp(' ');
  disp(['The filtering results are now displayed sequentially. ',...
       'Notice how the estimates gets more accurate when the filter gets on the right track. ',...
       'The green ellipse around the current estimate (little blue circle) reflects the filters ',...
       'confidence intervals of the position estimate.']);
  disp(' ');
  disp('<push any key to smooth the estimates with GHRTS>');
  if (~silent) pause; end
  clc;
  fprintf('Smoothing with GHRTS...');

  % Smoothing with URTS
  [SM1,SP1] = ghrts_smooth(MM,PP,A,Q,[],3);
  ghks_rmse1 = sqrt(mean((X(1,:)-SM1(1,:)).^2+(X(2,:)-SM1(2,:)).^2));
  ME1 = squeeze(SP1(1,1,:)+SP1(2,2,:));
  

  fprintf('Done!\n')
  disp(' ');
  disp(['Smoothing results of GHRTS are now displayed sequentially. ',...
        'Notice how the confidence ellipse gets even smaller now.']);
  disp(' ');
  disp('<push any key to proceed>');
  
  % Plot the smoothed estimates sequentially
  if ~silent
    M = SM1(:,1);  
    P = SP1(:,:,1);
    EST = M;
    cc = repmat(M(1:2),1,length(tt)) + ...
	 2*chol(P(1:2,1:2))'*[cos(tt);sin(tt)];
    h = plot(X(1,:),X(2,:),'r-',...
             M(1),M(2),'o',...
             EST(1,:),EST(2,:),'--',...
             cc(1,:),cc(2,:),'g-',...
             MM(1,:),MM(2,:),'b--',...
             S1(1),S1(2),'k^',S2(1),S2(2),'k^');
    legend('Location','NorthWest','Real trajectory','Current estimate','Smoothed trajectory',...
           'Confidence interval','Filter estimate','Positions of the sensors');
    title('Bearings Only Tracking with GHRTS.')
    axis([-1.5 1.5 -2.5 1.5]);
    EST = [];
    for k=size(Y,2):-steps:1
      M = SM1(:,k);  
      P = SP1(:,:,k);
      EST = SM1(:,end:-1:k);

      % Confidence ellipse
      cc = repmat(M(1:2),1,length(tt)) + ...
	 2*chol(P(1:2,1:2))'*[cos(tt);sin(tt)];

      % Update graphics
      set(h(2),'xdata',M(1)); set(h(2),'ydata',M(2));
      set(h(3),'xdata',EST(1,:)); set(h(3),'ydata',EST(2,:)); 
      set(h(4),'xdata',cc(1,:)); set(h(4),'ydata',cc(2,:)); 
      pause;
     end
  end
  

  %
  % Plot all the estimates together
  %
  if ~silent
    disp(' ')
    disp('<push any key to display all estimates together>')
    pause;
    clc;
    disp('All estimates are now displayed.')
  
    plot(X(1,:),X(2,:),'k-',...
         MM(1,:),MM(2,:),'b--',...
         SM1(1,:),SM1(2,:),'r-.',...
         S1(1),S1(2),'k^',S2(1),S2(2),'k^');
    axis([-1.5 1.5 -2.5 1.5]);
    legend('Real trajectory',...
           'GHKF estimate',...
           'GHRTS estimate',...
           'Positions of sensors',...
           'Location', 'NorthWest');
    title('Filtering and smoothing result with GHKF and GHRTS.');
    % Uncomment if you want to save an image
    %print -dpsc bot_demo_ghkf.ps
  end
  
  % Print RMSE
  disp(' ');
  disp('RMS errors:');
  fprintf('GHKF-RMSE  = %.3f  [%.3f]\n',ghkf_rmse,sqrt(mean(ME)));  
  fprintf('GHRTS-RMSE = %.4f [%.4f]\n',ghks_rmse1,sqrt(mean(ME1)));  