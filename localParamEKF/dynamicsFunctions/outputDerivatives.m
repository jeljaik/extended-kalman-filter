function dh = outputDerivatives(Xhat, model)
%    
% dh = rigidBodyOutputsDerivatives(...
%         Xhat(1) ,Xhat(2) ,Xhat(3),...  % v
%         Xhat(4) ,Xhat(5) ,Xhat(6),...  % w
%         Xhat(7) ,Xhat(8) ,Xhat(9),...  % f_1
%         Xhat(10),Xhat(11),Xhat(12),... % f_2
%         Xhat(13),Xhat(14),Xhat(15),... % mu_1 
%         Xhat(16),Xhat(17),Xhat(18),... % mu_2
%         Xhat(19),Xhat(20),Xhat(21),... % phi
%         model.I(1,1),model.I(2,2),model.I(3,3),model.m,model.g);
    
    dh = rigidBodyOutputsDerivatives(...
        Xhat(1) ,Xhat(2) ,Xhat(3),...  % v
        Xhat(4) ,Xhat(5) ,Xhat(6),...  % w
        Xhat(7) ,Xhat(8) ,Xhat(9),...  % f_o
        Xhat(10),Xhat(11),Xhat(12),... % mu_o
        Xhat(13),Xhat(14),Xhat(15),... % f_c 
        Xhat(16),Xhat(17),Xhat(18),... % mu_c
        Xhat(19),Xhat(20),Xhat(21),... % phi
        model.I(1,1),model.I(2,2),model.I(3,3),model.m,model.g);