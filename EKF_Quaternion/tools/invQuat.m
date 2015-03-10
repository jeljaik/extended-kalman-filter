function [ q ] = invQuat( q )
%INVQUAT Quaternion inversion

q = [-1*q(1:3);q(4)];


end

