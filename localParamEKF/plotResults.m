function plotResults(x, Xhat, P, t, index)

idx = 5; %Time 
stateVar = 1; % State variable from the state vector.

%% Estimated Forces 
stateVar = 1;
figure(index+1)
subplot(311)
shadedErrorBar(t(idx:end),Xhat(idx:end,stateVar),squeeze(2*sqrt(P(stateVar,stateVar,idx:end)))','b', 1);
hold on
plot(t(idx:end), x(idx:end,stateVar), '--b')
title('Total force acting on the body (x-component)');

subplot(312)
shadedErrorBar(t(idx:end),Xhat(idx:end,stateVar+1),squeeze(2*sqrt(P(stateVar+1,stateVar+1,idx:end)))','g',1);
hold on
plot(t(idx:end), x(idx:end,stateVar+1), '--g')
title('Total force acting on the body (y-component)');

subplot(313)
shadedErrorBar(t(idx:end),Xhat(idx:end,stateVar+2),squeeze(2*sqrt(P(stateVar+2,stateVar+2,idx:end)))','r',1);
hold on
plot(t(idx:end), x(idx:end,stateVar+2), '--r')
title('Total force acting on the body (z-component)');

%% Estimated Torques
figure(index+2)
subplot(311)
shadedErrorBar(t(idx:end),Xhat(idx:end,4),squeeze(2*sqrt(P(4,4,idx:end)))','b', 1);
hold on
plot(t(idx:end), x(idx:end,4), '--b')
title('Total torque acting on the body (x-component)');

subplot(312)
shadedErrorBar(t(idx:end),Xhat(idx:end,5),squeeze(2*sqrt(P(5,5,idx:end)))','g',1);
hold on
plot(t(idx:end), x(idx:end,5), '--g')
title('Total torque acting on the body (y-component)');

subplot(313)
shadedErrorBar(t(idx:end),Xhat(idx:end,6),squeeze(2*sqrt(P(6,6,idx:end)))','r',1);
hold on
plot(t(idx:end), x(idx:end,6), '--r')
title('Total torque acting on the body (z-component)');

%% Estimated Orientation
figure(3)
subplot(311)
plot(t(idx:end), x(idx:end,19),'--m'), hold on
shadedErrorBar(t(idx:end),Xhat(idx:end,19),squeeze(2*sqrt(P(19,19,idx:end)))','r',1);

subplot(312)
plot(t(idx:end), x(idx:end,20),'--m'), hold on
shadedErrorBar(t(idx:end),Xhat(idx:end,20),squeeze(2*sqrt(P(20,20,idx:end)))','r',1);

subplot(313)
plot(t(idx:end), x(idx:end,21),'--m'), hold on
shadedErrorBar(t(idx:end),Xhat(idx:end,21),squeeze(2*sqrt(P(21,21,idx:end)))','r',1);

% figure(index+3)
% subplot(221)
% shadedErrorBar(t,Xhat(:,13),squeeze(2*sqrt(P(13,13,:)))','b');
% hold on
% plot(t, x(:,13), '--b')
% 
% subplot(222)
% shadedErrorBar(t,Xhat(:,14),squeeze(2*sqrt(P(14,14,:)))','g');
% hold on
% plot(t, x(:,14), '--g')
% 
% subplot(223)
% shadedErrorBar(t,Xhat(:,15),squeeze(2*sqrt(P(15,15,:)))','r');
% hold on
% plot(t, x(:,15), '--r')
% 
% subplot(224)
% shadedErrorBar(t,Xhat(:,16),squeeze(2*sqrt(P(16,16,:)))','c');
% hold on
% plot(t, x(:,16), '--c')
% 
% for i = 1 : length(t)
%     z   (i,:) = q2dcm(   x(i, 13:16))*[0;0;1];
% end
% tr_param{2} = 10;
% for i = 1 : length(t)
%     % zhat(i,:) = q2dcm(Xhat(i, 13:16))*[0;0;1];
%     [zhat(i,:),Pz(:,:,i)] = ut_transform(Xhat(i, 13:16)',P(13:16,13:16,i),@computeVertical, [0;0;1], tr_param);
% end
% 
% figure(index+4)
% subplot(311)
% shadedErrorBar(t,zhat(:,1),squeeze(2*sqrt(Pz(1,1,:)))','b');
% %plot(t,zhat(:,1), 'b')
% hold on
% plot(t,z(:,1), 'b--')
% subplot(312)
% shadedErrorBar(t,zhat(:,2),squeeze(2*sqrt(Pz(2,2,:)))','r');
% %plot(t,zhat(:,2), 'r')
% hold on
% plot(t,z(:,2), 'r--')
% subplot(313)
% shadedErrorBar(t,zhat(:,3),squeeze(2*sqrt(Pz(3,3,:)))','g');
% %plot(t,zhat(:,3), 'g')
% hold on
% plot(t,z(:,3), 'g--')


end