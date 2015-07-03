function [com_R_imu] = optimalIMURotation(aPreProcMeanTest)

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

end


[min_val,idx]=min(T(:));
[row,col]=ind2sub(size(T),idx);

com_R_imu = euler2dcm([angle1+delRange(row),angle2+delRange(col),angle3]);

end