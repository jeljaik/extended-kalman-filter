function plotResults(x, Xhat, P, t, index)

figure(index+1)
subplot(311)
shadedErrorBar(t,Xhat(:,1),squeeze(2*sqrt(P(1,1,:)))','b', 1);
hold on
plot(t, x(:,1), '--b')
title('Total force acting on the body (x-component)');

subplot(312)
shadedErrorBar(t,Xhat(:,2),squeeze(2*sqrt(P(2,2,:)))','g',1);
hold on
plot(t, x(:,2), '--g')
title('Total force acting on the body (y-component)');

subplot(313)
shadedErrorBar(t,Xhat(:,3),squeeze(2*sqrt(P(3,3,:)))','r',1);
hold on
plot(t, x(:,3), '--r')
title('Total force acting on the body (z-component)');

figure(index+2)
subplot(311)
shadedErrorBar(t,Xhat(:,4),squeeze(2*sqrt(P(4,4,:)))','b', 1);
hold on
plot(t, x(:,4), '--b')
title('Total torque acting on the body (x-component)');

subplot(312)
shadedErrorBar(t,Xhat(:,5),squeeze(2*sqrt(P(5,5,:)))','g',1);
hold on
plot(t, x(:,5), '--g')
title('Total torque acting on the body (y-component)');

subplot(313)
shadedErrorBar(t,Xhat(:,6),squeeze(2*sqrt(P(6,6,:)))','r',1);
hold on
plot(t, x(:,6), '--r')
title('Total torque acting on the body (z-component)');

figure(3)
subplot(311)
plot(t, x(:,15),'--m'), hold on
shadedErrorBar(t,Xhat(:,15),squeeze(2*sqrt(P(15,15,:)))','r',1);


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