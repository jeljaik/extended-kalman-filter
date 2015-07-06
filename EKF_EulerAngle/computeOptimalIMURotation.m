%function T = acclTest(imuDel)

%imuDel1 = imuDel(1);
%imuDel2 = imuDel(2);
% aPreProcMeanTest = [-0.1779   -9.8068    0.1441]';%-[  -0.1898   -9.8090    0.1452]';
aPreProcMeanTest = [-0.2987   -9.8061    0.0681]';%[-0.4059 -9.8023 0.0636]';%[-0.2987   -9.8061    0.0681]'; %[-0.4725 -9.7984 0.0129]';%[-0.2703   -9.8090    0.0521]'
%omegaPreProcTest =  [-1.7197    0.8664    4.2135]';
%omegaRising = [-3.2080    0.3657    4.7032]';%[-0.0396    0.0185   -0.0064]';
omegaRisingPreProc= [ -0.9806    0.3700    2.5726]';
omegaOffset = [ -1.7742   -0.2684   3.1137]';

imuDel1 = 0.25*pi;%0.5*pi-0.35*pi;%-0.1*pi
imuDel2 = 0.0;%0.01*pi;%0.005*pi;

angle1 = pi/2;
angle2 = 1.5*pi;
angle3 = pi/2;

delRange = -0.1*pi:0.001*pi:0.1*pi;%-pi : 0.01*pi : 0;%-0.5*pi : 0.01*pi : 0.5*pi;
%diffG = zeros(length(delRange),length(delRange),3);
T = zeros(length(delRange), length(delRange));
ctr1=1;
ctr2=1;
for ctr1 = 1:length(delRange)%imuDel1 = delRange
    %ctr2 = 1;
    for ctr2 = 1:length(delRange)
        imuDel1 = delRange(ctr1);
        imuDel2 = delRange(ctr2);
        com_R_imu = euler2dcm([angle1+imuDel1,angle2+imuDel2,angle3]);

        a = (com_R_imu) * aPreProcMeanTest;

        %model.gRot = [-1;0;0];
        model.phi0 = [0,0.5*pi,0]';
        model.g  = -euler2dcm(model.phi0)'*[0 0 9.8]';
        %model.g = 9.81;

        diffG = a - [0 0 9.8]';%model.g*euler2dcm(model.phi0)*model.gRot;
        T(ctr1,ctr2) = diffG(1).^2+diffG(2).^2;
      %  ctr2 = ctr2+1;        
    end
    %ctr1 = ctr1+1;
    if(mod(ctr1,4) == 0)
        fprintf('%d, ',ctr1);
    end
    if(mod(ctr1,32) == 0)
        fprintf('\n ');
    end
end

%figure;
%surf(delRange,delRange,diffG(:,:,1).^2+diffG(:,:,2).^2);
%
%T = diffG(:,:,1).^2+diffG(:,:,2).^2;
%[t1,id1]=min(T) 
%[t2,id2]=min(t1)

figure;
surf(delRange,delRange,T);
%surf(delRange,delRange,diffG(:,:,2));
%plot(delRange,diffG(1:2,:));

[min_val,idx]=min(T(:))
[row,col]=ind2sub(size(T),idx)

com_R_imu = euler2dcm([angle1+delRange(row),angle2+delRange(col),angle3])

aDiff = (com_R_imu) * aPreProcMeanTest- [0; 0 ; 9.8];%model.g*euler2dcm(model.phi0)*model.gRot;
fprintf('aDiff : ');
disp(aDiff)

%fprintf('omegaRawMean : ');
%disp(com_R_imu * omegaRawMean)
fprintf('omegaOffset:');
disp(com_R_imu * omegaOffset);
fprintf('omegaRising : ');
disp(com_R_imu * omegaRisingPreProc)
fprintf('omegaRising - Offset : ');
disp(com_R_imu * (omegaRisingPreProc - omegaOffset))

 save('./IMUOffset_New.mat','com_R_imu');
