function [q] = convertToQuaternion(axisangle)
% Convert a axisangle rotation to a quaternion
q = [axisangle(1:3).*sin(axisangle(4)/2); cos(axisangle(4)/2)];    
end

