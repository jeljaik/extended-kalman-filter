function omega_hat = S(omega)

%Genova 03/08/2005
%Edited by Francesco Nori
%
% This function makes the following computation:
%
% omega_hat = [0            -omega(3, 1) omega(2, 1)
%              omega(3, 1)       0       -omega(1, 1)
%             -omega(2, 1)   omega(1, 1)          0];
%(see Murray-Lee-Sastry for details).

[m, n] = size(omega);
if (m==3)
	omega_hat = [0 -omega(3, 1) omega(2, 1); omega(3, 1) 0 -omega(1, 1); -omega(2, 1) omega(1, 1) 0];
else
	omega_hat = [0           -omega(6, 1)  omega(5, 1) omega(1,1);
	             omega(6, 1)           0  -omega(4, 1) omega(2,1);
	            -omega(5, 1)  omega(4, 1)           0  omega(3,1);
                     0                     0            0          0];
end
