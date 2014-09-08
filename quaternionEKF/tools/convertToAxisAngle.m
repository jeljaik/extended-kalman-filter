function [axisangle] = convertToAxisAngle(q)
t = wrapToPi(2*acos(q(4)));
k = q(1:3)./sin(t/2);
axisangle = [NormalizeV(k);t];
end