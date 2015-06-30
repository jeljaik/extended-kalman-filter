clear all;
close all;
%Connect to the INEMO device
[handle_dev pFD]=INEMO_Connection();
%End connection
acqSize=400;

%Gyroscope statistics
Offset=[-3.6982,-3.3570,-2.5909]';
var=[(0.7698/180*pi)^2 (0.4925/180*pi)^2 (0.5144/180*pi)^2]';

%Acquisition variables
GyroRate=zeros(3,acqSize);
Acc=zeros(3,acqSize);
Magn=zeros(3,acqSize);
Angles=zeros(3,acqSize);
AccF=zeros(3,acqSize);
MagnF=zeros(3,acqSize);


qUpdate=zeros(4,acqSize);
%Initial quaternion values
qUpdate(:,1)=[1 0 0 0]';

%Observation vector
qOsserv=zeros(4,acqSize);
qOsserv(:,1)=[1 0 0 0]';

%----KALMAN MATRIXES
Q1=[var(1,1)+var(2,1)+var(3,1) -var(1,1)+var(2,1)-var(3,1) -var(1,1)-var(2,1)+var(3,1) var(1,1)-var(2,1)-var(3,1)];
Q2=[-var(1,1)+var(2,1)-var(3,1) var(1,1)+var(2,1)+var(3,1) var(1,1)-var(2,1)-var(3,1) -var(1,1)-var(2,1)+var(3,1)];
Q3=[-var(1,1)-var(2,1)+var(3,1) var(1,1)-var(2,1)-var(3,1) var(1,1)+var(2,1)+var(3,1) -var(1,1)+var(2,1)-var(3,1)];
Q4=[var(1,1)-var(2,1)-var(3,1) -var(1,1)+var(2,1)-var(3,1) -var(1,1)+var(2,1)-var(3,1) var(1,1)+var(2,1)+var(3,1)];
Qmatrix=[Q1;Q2;Q3;Q4];

H=eye(4,4);

sigmaR=[0.1 0.1 0.1 0.1]';
R=[sigmaR(1,1) 0 0 0;0 sigmaR(2,1) 0 0;0 0 sigmaR(3,1) 0;0 0 0 sigmaR(4,1)];

qPredicted=zeros(4,acqSize);
qPredicted(:,1)=[0.5 0.5 0.5 0.5]';
P_Update=eye(4,4)*0.001;
%----------
t=[0];

i=1;
dt=0;

[bAcc,aAcc] = butter(3,0.0075,'low');
[bMagn,aMagn] = butter(2,0.06,'low');

magnF_Length=13;
accF_Length=13;

%Bring up the filters
while(i<=accF_Length+4)
    if(i>1)
        dt = toc(t0);
        t=[t t(length(t))+dt];
    end
    [errre pFD]=calllib('iNEMO2_SDK','INEMO2_GetDataSample',handle_dev,pFD);
    t0 = tic;

        %----------
        pause(0.01)
        %---------
        
    Acc(1,i)=pFD.Accelerometer.X;
    Acc(2,i)=pFD.Accelerometer.Y;
    Acc(3,i)=pFD.Accelerometer.Z;
    Magn(1,i)=pFD.Magnetometer.X;
    Magn(2,i)=pFD.Magnetometer.Y;
    Magn(3,i)=pFD.Magnetometer.Z;
    GyroRate(1,i)=((pFD.Gyroscope.X-Offset(1,1))/180)*pi;
    GyroRate(2,i)=((pFD.Gyroscope.Y-Offset(2,1))/180)*pi;
    GyroRate(3,i)=((pFD.Gyroscope.Z-Offset(3,1))/180)*pi;
    
    Acc(:,i)=Acc(:,i)/norm(Acc(:,i));
    Magn(:,i)=Magn(:,i)/norm(Magn(:,i));
    if(i<=accF_Length)
        AccF(:,i)=MyFilter(bAcc,aAcc,Acc(:,:));
    else
        AccF(:,i)=MyFilter(bAcc,aAcc,Acc(:,i-accF_Length:i));
    end
    if(i<=magnF_Length)
        MagnF(:,i)=MyFilter(bMagn,aMagn,Magn(:,:));
    else
        MagnF(:,i)=MyFilter(bMagn,aMagn,Magn(:,i-magnF_Length:i));
    end
    MagnF(:,i)=MagnF(:,i)/norm(MagnF(:,i));
    AccF(:,i)=AccF(:,i)/norm(AccF(:,i));
    i=i+1;
    qPredicted(:,i)=[1 0 0 0]';
    qUpdate(:,i)=[1 0 0 0]';
    qOsserv(:,i)=[1 0 0 0]';
end

while(i<=acqSize)
    if(i>2)
        dt = toc(t0);
        t=[t t(length(t))+dt];
    end
    
    %----Acquisition
        [errre pFD]=calllib('iNEMO2_SDK','INEMO2_GetDataSample',handle_dev,pFD);
        t0 = tic;

        %----------
        pause(0.01)
        %---------
        
    Acc(1,i)=pFD.Accelerometer.X;
    Acc(2,i)=pFD.Accelerometer.Y;
    Acc(3,i)=pFD.Accelerometer.Z;
    Magn(1,i)=pFD.Magnetometer.X;
    Magn(2,i)=pFD.Magnetometer.Y;
    Magn(3,i)=pFD.Magnetometer.Z;
    GyroRate(1,i)=((pFD.Gyroscope.X-Offset(1,1))/180)*pi;
    GyroRate(2,i)=((pFD.Gyroscope.Y-Offset(2,1))/180)*pi;
    GyroRate(3,i)=((pFD.Gyroscope.Z-Offset(3,1))/180)*pi;
    
    %Normalization and filtering
    Acc(:,i)=Acc(:,i)/norm(Acc(:,i));
    Magn(:,i)=Magn(:,i)/norm(Magn(:,i));
    
    AccF(:,i)=MyFilter(bAcc,aAcc,Acc(:,i-accF_Length:i));
    MagnF(:,i)=MyFilter(bMagn,aMagn,Magn(:,i-magnF_Length:i));
    
    MagnF(:,i)=MagnF(:,i)/norm(MagnF(:,i));
    AccF(:,i)=AccF(:,i)/norm(AccF(:,i));
    %----End Acquisition
    
    %OBSERVATION COMPUTING
    %Gauss Newton step 
    %qOss=GaussNewtonMethod(qUpdate(2,i-1),qUpdate(3,i-1),qUpdate(4,i-1),qUpdate(1,i-1),AccF(:,i),MagnF(:,i),MagnF(:,17));
    %qOsserv(:,i)=[qOss(4,1); qOss(2:4,1)];
    
    %Gradient Descent
    dq=0.5*(QuaternionProduct(qUpdate(:,i-1),[0 GyroRate(1,i) GyroRate(2,i) GyroRate(3,i)]'));
    mu=norm(dq)*0.6*(1/(norm(dq)^(71/100)));
    %mu=5*norm(dq)*dt;
    qOsserv(:,i)=GradientDescent(AccF(:,i),MagnF(:,i),qUpdate(:,i-1),mu);
    
    qOsserv(:,i)=qOsserv(:,i)/norm(qOsserv(:,i));
    %END OSSERVATION COMPUTING
    
    %KALMAN FILTERING
    
    %F matrix computing
    F1=[1 -dt/2*GyroRate(1,i) -dt/2*GyroRate(2,i) -dt/2*GyroRate(3,i)];
    F2=[dt/2*GyroRate(1,i) 1 dt/2*GyroRate(3,i) -dt/2*GyroRate(2,i)];
    F3=[dt/2*GyroRate(2,i) -dt/2*GyroRate(3,i) 1 dt/2*GyroRate(1,i)];
    F4=[-dt/2*GyroRate(3,i) dt/2*GyroRate(2,i) -dt/2*GyroRate(1,i) 1];
    
    F=[F1;F2;F3;F4];
    qPredicted(:,i)=F*qUpdate(:,i-1);
    
    Q=(dt/2)^2*Qmatrix;
    
    P_Predicted=F*P_Update*F'+Q;
    
    K=P_Predicted*H'*(H*P_Predicted*H'+R)^-1;
    
    qUpdate(:,i)=qPredicted(:,i)+K*(qOsserv(:,i)-H*qPredicted(:,i));
    qUpdate(:,i)=qUpdate(:,i)/norm(qUpdate(:,i));
    
    P_Update=(eye(4,4)-K*H)*P_Predicted;
    Angles(:,i)=GetAnglesFromQuaternion(qUpdate(:,i));
    
    
    %END KALMAN FILTERING
    i=i+1;
end
figure;
    subplot(3,1,1);plot(t,Acc(1,:),'b',t,AccF(1,:),'r',t,Magn(1,:),'g',t,MagnF(1,:),'c');legend('Acc X','Acc X Filtered','Magn X','Magn X Filtered');grid;xlabel('time (sec)');ylabel('angle (deg)');
    subplot(3,1,2);plot(t,Acc(2,:),'b',t,AccF(2,:),'r',t,Magn(2,:),'g',t,MagnF(2,:),'c');legend('Acc Y','Acc Y Filtered','Magn Y','Magn Y Filtered');grid;xlabel('time (sec)');ylabel('angle (deg)');
    subplot(3,1,3);plot(t,Acc(3,:),'b',t,AccF(3,:),'r',t,Magn(3,:),'g',t,MagnF(3,:),'c');legend('Acc Z','Acc Z Filtered','Magn Z','Magn Z Fitlered');grid;xlabel('time (sec)');ylabel('angle (deg)');


figure;
    subplot(4,1,1);plot(t,qOsserv(1,1:acqSize));grid;legend('q0 Observed');
    subplot(4,1,2);plot(t,qOsserv(2,1:acqSize));grid;legend('q1 Observed');
    subplot(4,1,3);plot(t,qOsserv(3,1:acqSize));grid;legend('q2 Observed');
    subplot(4,1,4);plot(t,qOsserv(4,1:acqSize));grid;legend('q3 Observed');

figure;
    subplot(4,1,1);plot(t,qUpdate(1,1:acqSize));grid;legend('q0 Estimated');
    subplot(4,1,2);plot(t,qUpdate(2,1:acqSize));grid;legend('q1 Estimated');
    subplot(4,1,3);plot(t,qUpdate(3,1:acqSize));grid;legend('q2 Estimated');
    subplot(4,1,4);plot(t,qUpdate(4,1:acqSize));grid;legend('q3 Estimated');
    
figure;
    subplot(3,1,1);plot(t,Angles(1,1:acqSize));grid;legend('Roll');
   subplot(3,1,2);plot(t,Angles(2,1:acqSize));grid;legend('Pitch');
   subplot(3,1,3);plot(t,Angles(3,1:acqSize));grid;legend('Yaw');


INEMO_Disconnection(handle_dev);
