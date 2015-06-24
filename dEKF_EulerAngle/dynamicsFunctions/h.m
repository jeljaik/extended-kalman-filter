function y = h(t, x, model)

dx = zeros(size(x'));
w  = zeros(size(x'));
for i = 1:length(t)
    [dx(:,i), w(:,i)] = rigidBodyOutput(t(i),x(i,:)',model);
end
dx = dx';
w  = w';

y         = [dx(:, 1:3), w]; 
end

