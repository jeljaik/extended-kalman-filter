function z = computeVertical(q, g)
z = q2dcm(q)*g;
end