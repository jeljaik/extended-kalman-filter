function [ flag ] = rayTriangleIntersectionDir( d, p0, p1, p2, out)
%RAYTRIANGLEINTERSECTIONDIR Test whether a ray direction is outward or
%inward a plane defined by 3 points and an outward vector
%   flag: 0 outward, 1 inward, 2 parallel
    e1 = p1-p0;
    e2 = p2-p0;
    nn = cross(e1,e2); % normal to the plane defined by the triangle
    
    % the plane equation is: nn*X-nn*p0 = 0
    k = dot(nn,p0);
    testOut = dot(nn,p0+out)-k;
    testRay = dot(nn,p0+d)-k;
    res = testOut*testRay;
    
    if(res>0)
        % the ray and the outward vector point on the same plane side
        flag=0;
    elseif(res<0)
        flag=1;
    else
        % the ray is parallel to the plane
        flag=2;
    end    
end

