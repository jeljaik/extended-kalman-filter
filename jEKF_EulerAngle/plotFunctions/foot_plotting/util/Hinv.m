function [ H_inv ] = Hinv( H )
%HINV Return the inverse homogeneous transformation matrix
    H_inv = eye(4);
    H_inv(1:3,1:3) = H(1:3,1:3)';               % rotation part
    H_inv(1:3,4) = -H_inv(1:3,1:3) * H(1:3,4);  % translation part
end
