function [ qx ] = skew( q )
%SKEW Function that gets a skew matrix from a quaternion

qx = [  0   -q(3)  q(2);
       q(3)   0   -q(1);
      -q(2)  q(1)   0 ];

end

