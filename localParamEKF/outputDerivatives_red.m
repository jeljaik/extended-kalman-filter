function dh = outputDerivatives_red(Xhat, model)
   
dh = rigidBodyOutputsDerivatives_red(...
        Xhat(1) ,Xhat(2) ,Xhat(3),...  % v
        Xhat(4) ,Xhat(5) ,Xhat(6),...  % w
        Xhat(7) ,Xhat(8) ,Xhat(9),...  % phi
        model.I(1,1),model.I(2,2),model.I(3,3),model.m,model.g);