function [ points_W, H_6S ] = ftSensor2elbowTransformation( rightArm, points_S)
%FTSENSOR2ELBOWTRANSFORMATION Project a list of points from the F/T
% sensor reference frame to the elbow reference frame (attached to the
% upperarm cover).
    
    % compute the transformation from the F/T sensor to the elbow
    H_6S = zeros(4);
    if(rightArm)
        H_6S(1,1)=-1; H_6S(3,2)= 1; H_6S(2,3)=1; H_6S(2,4)=-0.08428;  H_6S(4,4)=1;
    else
        H_6S(1,1)=1; H_6S(2,3)=-1; H_6S(3,2)=1; H_6S(2,4)=0.08428; H_6S(4,4)=1;
    end
    
    % if the input points are not represented in homogeneous coordinates
    % add the fourth column
    if(size(points_S,2)==3)
        points_S(:,4) = 1;
    end
    
    % project the points from sensor to wrist reference frame
    points_W = zeros(size(points_S));
    for i=1:size(points_S,1)
       points_W(i,:) = H_6S * points_S(i,:)';
    end
    if(size(points_W,1)>0)
        points_W = points_W(:,1:3);
    end
end