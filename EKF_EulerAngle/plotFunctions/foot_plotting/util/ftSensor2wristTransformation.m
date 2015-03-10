function [ points_W, H_8S ] = ftSensor2wristTransformation( rightArm, points_S, joint3, joint4 )
%FTSENSOR2WRISTTRANSFORMATION Project a list of points from the F/T
% sensor reference frame to the wrist reference frame (attached to the forearm).
% The joint3 and joint4 input angle are in degrees.

    % compute the transformation from the elbow to the wrist
    theta3 = joint3*pi/180;
    theta4 = joint4*pi/180;
    if(rightArm)
        H_67 = evalDHMatrix( 0.015,      0,     pi/2,  theta3); 
        H_78 = evalDHMatrix(   0,   -0.1373,     pi/2, theta4-pi/2);
    else
        H_67 = evalDHMatrix( -0.015,      0,     pi/2,  theta3); 
        H_78 = evalDHMatrix(   0,   0.1373,     pi/2, theta4-pi/2);
    end
    H_86 = Hinv(H_67*H_78);
    
    % compute the transformation from the F/T sensor to the elbow
    H_6S = zeros(4);
    if(rightArm)
        H_6S(1,1)=-1; H_6S(3,2)= 1; H_6S(2,3)=1; H_6S(2,4)=-0.08428;  H_6S(4,4)=1;
    else
        H_6S(1,1)=1; H_6S(2,3)=-1; H_6S(3,2)=1; H_6S(2,4)=0.08428; H_6S(4,4)=1;
    end
    H_8S = H_86 * H_6S;     % homogeneous transformation from wrist (8) to F/T sensor
    
    % if the input points are not represented in homogeneous coordinates
    % add the fourth column
    if(size(points_S,2)==3)
        points_S(:,4) = 1;
    end
    
    % project the points from sensor to wrist reference frame
    points_W = zeros(size(points_S));
    for i=1:size(points_S,1)
       points_W(i,:) = H_8S * points_S(i,:)';
    end
    if(size(points_W,1)>0)
        points_W = points_W(:,1:3);
    end
end
