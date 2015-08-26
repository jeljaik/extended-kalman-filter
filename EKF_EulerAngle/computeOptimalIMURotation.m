% script computing the IMU-Body CoM rototranslation
% Assumes that IMU is attached to ankle with small variations about z and y axes
% Given the pre processing mean of the Accelerometer readings, the desired
% rototranslation is computed by : 
% com_R_imu = argmin || com_R_imu*a_imu - [0,0,-9.8]' ||
% obviously it assumes the optimal rototranslation to the CoM frame will
% give a calibration dataset mean acceleration that is identical to
% [0;0;-9.8]
% Arguments : 
% dataset - For direct loading saved rotation, 1 for old dataset, 2 - for new dataset, empty for recomputation 

function [com_R_imu] = computeOptimalIMURotation( datasetAge,aCalib, plots, verboseMode)
tic;
fName = sprintf('IMUOffset_data%d.mat',datasetAge);
if (nargin == 1 && exist(fName,'file')~=0)
   load(fName);
   fprintf('Loading com_R_imu from pre computed matrix\n');
else

    fprintf('Computing com_R_imu from calibration average accelerometer measurement\n');

    %% setting up parameter ranges
    % angles assigned from intiution on IMU attachment to ankle
    angle1 = pi/2; angle2 = -pi/2;   angle3 = -pi/2;
    delRange = -0.1*pi:0.001*pi:0.1*pi;
    
    Jsurf = zeros(length(delRange), length(delRange));
    
    %% Iterating through the parameter range computing cost function
    for ctr1 = 1:length(delRange)
        for ctr2 = 1:length(delRange)
            imuDel1 = delRange(ctr1);imuDel2 = delRange(ctr2);
            com_R_imu = euler2dcm([angle1+imuDel1,angle2+imuDel2,angle3])';
            a = (com_R_imu) * aCalib;
            J = norm(a - [0 0 -9.8]');%model.g*euler2dcm(model.phi0)*model.gRot;
            Jsurf(ctr1,ctr2) = J;%diffG(1).^2+diffG(2).^2;
        end
        
%         if(strcmp(verboseMode,'on')==1)
%             if(mod(ctr1,5) == 0)
%                 fprintf('%d..',ctr1);
%             end
%             if(mod(ctr1,40) == 0)
%                 fprintf('\n');
%             end
%         end
    end

    if(strcmp(plots,'MakePlots') == 1)
        figure;
        surf(delRange,delRange,Jsurf);
    end
    
    [min_val,idx]=min(Jsurf(:));
    [row,col]=ind2sub(size(Jsurf),idx);

    com_R_imu = euler2dcm([angle1+delRange(row),angle2+delRange(col),angle3])';
    
    
    if(strcmp(verboseMode,'on')==1)
        fprintf('com_R_imu : \n');
        disp(com_R_imu);
        
        aDiff = (com_R_imu) * aCalib - [0; 0 ; -9.8];
    
        fprintf('aDiff : \n');
        disp(aDiff);
        fprintf(' Processing time : %d secs\n',toc());
    end
    
    %% saving computed result in local folder
    save(sprintf('./IMUOffset_data%d.mat',datasetAge),'com_R_imu');
   end

end