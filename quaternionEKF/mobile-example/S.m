function [ S ] = S( w )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
wx = w(1); wy = w(2); wz = w(3);

S = [ 0  -wz  wy;
      wz  0  -wx;
     -wy  wx   0];

end

