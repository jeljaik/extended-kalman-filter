function [ c ] = convertToMatrix( q )
%CONVERTTOMATRIX Summary of this function goes here
%   Left-hand-rule rotation matrix because of the current quaternion convention
   
c = eye(3) - 2*q(4)*skew(q) + 2*(skew(q)*skew(q));
end

