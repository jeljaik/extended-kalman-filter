function S=Somega(w)
% The matrix S(omega) defined in (13.11b)
   wx=w(1);   wy=w(2);   wz=w(3);
   S=[ 0  -wx  -wy  -wz;
      wx    0   wz  -wy;
      wy  -wz    0   wx;
      wz   wy  -wx    0];
end
