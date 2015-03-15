function euler = q2euler(q)
% Q2EULER  Convert quaternions to Euler angles

  euler = zeros(3, size(q, 2));

  xzpwy = q(2, :).*q(4, :) + q(1, :).*q(3, :);

	IN = xzpwy+sqrt(eps)>0.5;  % Handle the north pole
  euler(1, IN) = 2*atan2(q(2, IN), q(1, IN));
  IS = xzpwy-sqrt(eps)<0.5;  % Handle the south pole
  euler(1, IS) = -2*atan2(q(2, IS), q(1, IS));

  I = ~(IN | IS);  % Handle the default case
  euler(1, I) = atan2(-2*(q(1, I).*q(3, I) - q(1, I).*q(4, I)),...
                      1-2*(q(3, I).^2 + q(4, I).^2));
  euler(3, I) = atan2(2*(q(3, I).*q(4, I) - q(1, I).*q(2, I)),...
                      1-2*(q(2, I).^2 + q(3, I).^2));

	euler(2, :) = asin(2*xzpwy);

  euler(1, :) = rem(euler(1, :), 2*pi);
end
