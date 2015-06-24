function plotResultsOutput(Xhat, P, tK, yM)

figPreN = figure()-1;
idx = 1; %Time 
index = 0;
%stateVar = 1; % State variable from the state vector.
tMin = 0.0;
%% Estimated Forces 
stateVar = 7;
figure(1+figPreN)
subplot(3,2,1)
shadedErrorBar(tK(idx:end),Xhat(idx:end,stateVar),squeeze(2*sqrt(P(stateVar,stateVar,idx:end)))','b', 1);
hold on
plot(tK(idx:end), yM(idx:end,stateVar), '--b')
title('Estimation of force f_o');
axis tight;
xlabel('Time t(sec)');
ylabel('E(f _x) N');
a = axis();
axis([tMin a(2) a(3) a(4)]);

subplot(3,2,3);
shadedErrorBar(tK(idx:end),Xhat(idx:end,stateVar+1),squeeze(2*sqrt(P(stateVar+1,stateVar+1,idx:end)))','g',1);
hold on
plot(tK(idx:end), yM(idx:end,stateVar+1), '--g')
%title('Total force acting on the body (y-component)');
axis tight;
xlabel('Time t(sec)');
ylabel('E(f _y) N');
a = axis();
axis([tMin a(2) a(3) a(4)]);

subplot(3,2,5)
shadedErrorBar(tK(idx:end),Xhat(idx:end,stateVar+2),squeeze(2*sqrt(P(stateVar+2,stateVar+2,idx:end)))','r',1);
hold on
plot(tK(idx:end), yM(idx:end,stateVar+2), '--r')
%title('Total force acting on the body (z-component)');
axis tight;
xlabel('Time t(sec)');
ylabel('E(f _z) N');
a = axis();
axis([tMin a(2) a(3) a(4)]);


subplot(3,2,2)
shadedErrorBar(tK(idx:end),Xhat(idx:end,stateVar+3),squeeze(2*sqrt(P(stateVar+3,stateVar+3,idx:end)))','b', 1);
hold on
plot(tK(idx:end), yM(idx:end,stateVar+3), '--b')
title('Estimation of force f_c');
axis tight;
xlabel('Time t(sec)');
ylabel('E(f _x) N');
a = axis();
axis([tMin a(2) a(3) a(4)]);

subplot(3,2,4);
shadedErrorBar(tK(idx:end),Xhat(idx:end,stateVar+4),squeeze(2*sqrt(P(stateVar+4,stateVar+4,idx:end)))','g',1);
hold on
plot(tK(idx:end), yM(idx:end,stateVar+4), '--g')
%title('Total force acting on the body (y-component)');
axis tight;
xlabel('Time t(sec)');
ylabel('E(f _y) N');
a = axis();
axis([tMin a(2) a(3) a(4)]);

subplot(3,2,6)
shadedErrorBar(tK(idx:end),Xhat(idx:end,stateVar+5),squeeze(2*sqrt(P(stateVar+5,stateVar+5,idx:end)))','r',1);
hold on
plot(tK(idx:end), yM(idx:end,stateVar+5), '--r')
%title('Total force acting on the body (z-component)');
axis tight;
xlabel('Time t(sec)');
ylabel('E(f _z) N');
a = axis();
axis([tMin a(2) a(3) a(4)]);



%% Estimated Torques 
stateVar = 13;
figure(index+2+figPreN)
subplot(3,2,1)
shadedErrorBar(tK(idx:end),Xhat(idx:end,stateVar),squeeze(2*sqrt(P(stateVar,stateVar,idx:end)))','b', 1);
hold on
plot(tK(idx:end), yM(idx:end,stateVar), '--b')
title('Estimation of torque \mu_o');
axis tight;
xlabel('Time t(sec)');
ylabel('E(\mu _x) Nm');
a = axis();
axis([tMin a(2) a(3) a(4)]);

subplot(3,2,3);
shadedErrorBar(tK(idx:end),Xhat(idx:end,stateVar+1),squeeze(2*sqrt(P(stateVar+1,stateVar+1,idx:end)))','g',1);
hold on
plot(tK(idx:end), yM(idx:end,stateVar+1), '--g')
%title('Total force acting on the body (y-component)');
axis tight;
xlabel('Time t(sec)');
ylabel('E(\mu _y) N(m)');
a = axis();
axis([tMin a(2) a(3) a(4)]);

subplot(3,2,5)
shadedErrorBar(tK(idx:end),Xhat(idx:end,stateVar+2),squeeze(2*sqrt(P(stateVar+2,stateVar+2,idx:end)))','r',1);
hold on
plot(tK(idx:end), yM(idx:end,stateVar+2), '--r')
axis tight;
xlabel('Time t(sec)');
ylabel('E(\mu _z) N(m)');
a = axis();
axis([tMin a(2) a(3) a(4)]);


subplot(3,2,2)
shadedErrorBar(tK(idx:end),Xhat(idx:end,stateVar+3),squeeze(2*sqrt(P(stateVar+3,stateVar+3,idx:end)))','b', 1);
hold on
plot(tK(idx:end), yM(idx:end,stateVar+3), '--b')
title('Estimation of torque \mu_c');
axis tight;
xlabel('Time t(sec)');
ylabel('E(\mu _x) Nm');
a = axis();
axis([tMin a(2) a(3) a(4)]);

subplot(3,2,4);
shadedErrorBar(tK(idx:end),Xhat(idx:end,stateVar+4),squeeze(2*sqrt(P(stateVar+4,stateVar+4,idx:end)))','g',1);
hold on
plot(tK(idx:end), yM(idx:end,stateVar+4), '--g')
%title('Total force acting on the body (y-component)');
axis tight;
xlabel('Time t(sec)');
ylabel('E(\mu _y) Nm');a = axis();
axis([tMin a(2) a(3) a(4)]);

subplot(3,2,6)
shadedErrorBar(tK(idx:end),Xhat(idx:end,stateVar+5),squeeze(2*sqrt(P(stateVar+5,stateVar+5,idx:end)))','r',1);
hold on
plot(tK(idx:end), yM(idx:end,stateVar+5), '--r')
axis tight;
xlabel('Time t(sec)');
ylabel('E(\mu _z) Nm');
a = axis();
axis([tMin a(2) a(3) a(4)]);





%% Estimated Velocities 
stateVar = 1;
figure(index+3+figPreN)
subplot(3,2,1)
shadedErrorBar(tK(idx:end),Xhat(idx:end,stateVar),squeeze(2*sqrt(P(stateVar,stateVar,idx:end)))','b', 1);
hold on
%plot(tK(idx:end), yM(idx:end,stateVar), '--b')
title('Estimation of Translational Velocity v^B');
axis tight;
xlabel('Time t(sec)');
ylabel('E(v _x) Nm');
a = axis();
axis([tMin a(2) a(3) a(4)]);

subplot(3,2,3);
shadedErrorBar(tK(idx:end),Xhat(idx:end,stateVar+1),squeeze(2*sqrt(P(stateVar+1,stateVar+1,idx:end)))','g',1);
hold on
%plot(tK(idx:end), yM(idx:end,stateVar+1), '--g')
%title('Total force acting on the body (y-component)');
axis tight;
xlabel('Time t(sec)');
ylabel('E(v _y) N(m)');
a = axis();
axis([tMin a(2) a(3) a(4)]);

subplot(3,2,5)
shadedErrorBar(tK(idx:end),Xhat(idx:end,stateVar+2),squeeze(2*sqrt(P(stateVar+2,stateVar+2,idx:end)))','r',1);
hold on
%plot(tK(idx:end), yM(idx:end,stateVar+2), '--r')
axis tight;
xlabel('Time t(sec)');
ylabel('E(v _z) N(m)');
a = axis();
axis([tMin a(2) a(3) a(4)]);


subplot(3,2,2)
shadedErrorBar(tK(idx:end),Xhat(idx:end,stateVar+3),squeeze(2*sqrt(P(stateVar+3,stateVar+3,idx:end)))','b', 1);
hold on
plot(tK(idx:end), yM(idx:end,stateVar+3), '--b')
title('Estimation of Angular Velocity \omega ^B');
axis tight;
xlabel('Time t(sec)');
ylabel('E(\omega _1) Nm');
a = axis();
axis([tMin a(2) a(3) a(4)]);

subplot(3,2,4);
shadedErrorBar(tK(idx:end),Xhat(idx:end,stateVar+4),squeeze(2*sqrt(P(stateVar+4,stateVar+4,idx:end)))','g',1);
hold on
plot(tK(idx:end), yM(idx:end,stateVar+4), '--g')
%title('Total force acting on the body (y-component)');
axis tight;
xlabel('Time t(sec)');
ylabel('E(\omega _2) Nm');a = axis();
axis([tMin a(2) a(3) a(4)]);

subplot(3,2,6)
shadedErrorBar(tK(idx:end),Xhat(idx:end,stateVar+5),squeeze(2*sqrt(P(stateVar+5,stateVar+5,idx:end)))','r',1);
hold on
plot(tK(idx:end), yM(idx:end,stateVar+5), '--r')
axis tight;
xlabel('Time t(sec)');
ylabel('E(\omega _3) Nm');
a = axis();
axis([tMin a(2) a(3) a(4)]);

%% Estimated Orientation
stateVar = 1;
figure(index+4+figPreN)

subplot(3,1,1)
%plot(t(idx:end), x(idx:end,19),'--m'), hold on
shadedErrorBar(tK(idx:end),Xhat(idx:end,19),squeeze(2*sqrt(P(19,19,idx:end)))','r',1);
axis tight;
title('Orientation Estimate');
axis tight;
xlabel('Time t(sec)');
ylabel('E(\phi (t) rads');
a = axis();
axis([tMin a(2) a(3) a(4)]);

subplot(3,1,2)
%plot(t(idx:end), x(idx:end,20),'--m'), hold on
shadedErrorBar(tK(idx:end),Xhat(idx:end,20),squeeze(2*sqrt(P(20,20,idx:end)))','r',1);
axis tight;
xlabel('Time t(sec)');
ylabel('E(\phi (t) rads');
a = axis();
axis([tMin a(2) a(3) a(4)]);

%% Estimated Orientation
stateVar = 1;
figure(index+4+figPreN)

subplot(3,1,1)
%plot(t(idx:end), x(idx:end,19),'--m'), hold on
shadedErrorBar(tK(idx:end),Xhat(idx:end,19),squeeze(2*sqrt(P(19,19,idx:end)))','r',1);
axis tight;
title('Orientation Estimate phi(t)');
axis tight;
xlabel('Time t(sec)');
ylabel('E(\alpha (t) rads');
a = axis();
axis([tMin a(2) a(3) a(4)]);

subplot(3,1,2)
%plot(t(idx:end), x(idx:end,20),'--m'), hold on
shadedErrorBar(tK(idx:end),Xhat(idx:end,20),squeeze(2*sqrt(P(20,20,idx:end)))','r',1);
axis tight;
xlabel('Time t(sec)');
ylabel('E(\upsilon (t) rads');
a = axis();
axis([tMin a(2) a(3) a(4)]);


subplot(3,1,3)
%plot(t(idx:end), x(idx:end,21),'--m'), hold on
shadedErrorBar(tK(idx:end),Xhat(idx:end,21),squeeze(2*sqrt(P(21,21,idx:end)))','r',1);
axis tight;
xlabel('Time t(sec)');
ylabel('E(\psi (t) rads');
a = axis();
axis([tMin a(2) a(3) a(4)]);

subplot(3,1,3)
%plot(t(idx:end), x(idx:end,21),'--m'), hold on
shadedErrorBar(tK(idx:end),Xhat(idx:end,21),squeeze(2*sqrt(P(21,21,idx:end)))','r',1);
axis tight;
xlabel('Time t(sec)');
ylabel('E(\phi (t) rads');
a = axis();
axis([tMin a(2) a(3) a(4)]);

% 
% %% Estimated Orientation
% figure(index+3+figPreN)
% subplot(3,1,1)
% plot(t(idx:end), x(idx:end,19),'--m'), hold on
% shadedErrorBar(tK(idx:end),Xhat(idx:end,19),squeeze(2*sqrt(P(19,19,idx:end)))','r',1);
% axis tight;
% title('Orientation Estimate');
% axis tight;
% xlabel('Time t(sec)');
% ylabel('E(\phi (t) rads');
% a = axis();
% axis([tMin a(2) a(3) a(4)]);
% 
% subplot(3,1,2)
% plot(t(idx:end), x(idx:end,20),'--m'), hold on
% shadedErrorBar(tK(idx:end),Xhat(idx:end,20),squeeze(2*sqrt(P(20,20,idx:end)))','r',1);
% axis tight;
% xlabel('Time t(sec)');
% ylabel('E(\phi (t) rads');
% a = axis();
% axis([tMin a(2) a(3) a(4)]);
% 
% 
% subplot(3,1,3)
% plot(t(idx:end), x(idx:end,21),'--m'), hold on
% shadedErrorBar(tK(idx:end),Xhat(idx:end,21),squeeze(2*sqrt(P(21,21,idx:end)))','r',1);
% axis tight;
% xlabel('Time t(sec)');
% ylabel('E(\phi (t) rads');
% a = axis();
% axis([tMin a(2) a(3) a(4)]);


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