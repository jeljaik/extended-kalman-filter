function dc = paramOutputs(Xhat, model)


% function dc_dw = paramOutputsDerivatives(v_Bx,v_By,v_Bz,omega_Bx,omega_By,omega_Bz,f_B_ox,f_B_oy,f_B_oz,mu_B_ox,mu_B_oy,mu_B_oz,f_B_cx,f_B_cy,f_B_cz,mu_B_cx,mu_B_cy,mu_B_cz,phi1,phi2,phi3,K_Bxx,K_Bxy,K_Bxz,K_Byx,K_Byy,K_Byz,K_Bzx,K_Bzy,K_Bzz,I_Bxx,I_Bxy,I_Bxz,I_Byy,I_Byz,I_Bzz,m,g,gRot1,gRot2,gRot3)


dc = paramOutputsDerivatives(...
        Xhat(1) ,Xhat(2) ,Xhat(3),...  % v
        Xhat(4) ,Xhat(5) ,Xhat(6),...  % w
        Xhat(7) ,Xhat(8) ,Xhat(9),...  % f_o
        Xhat(10),Xhat(11),Xhat(12),... % mu_o
        Xhat(13),Xhat(14),Xhat(15),... % f_c 
        Xhat(16),Xhat(17),Xhat(18),... % mu_c
        Xhat(19),Xhat(20),Xhat(21),... % phi
        model.K(1,1) ,model.K(1,2) ,model.K(1,3),...  
        model.K(2,1) ,model.K(2,2) ,model.K(2,3),... 
        model.K(3,1) ,model.K(3,2) ,model.K(3,3),... 
        model.I(1,1), model.I(1,2),model.I(1,3),model.I(2,2),model.I(2,3),model.I(3,3),model.m,model.g,model.gRot(1),model.gRot(2),model.gRot(3));