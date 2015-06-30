function [n] = GaussNewtonMethod(q,Acc,Magn)
    %Calibration Values
    Magn(1,1)=Magn(1,1)+0.38;
    Magn(2,1)=Magn(2,1)*1.1;
    Magn(3,1)=Magn(3,1)-0.08;
    %Compute the new step quaternions by mean of the Gauss-Newton method
    a=q(2,1);
    b=q(3,1);
    c=q(4,1);
    d=q(1,1);
    
    i=1;
    n_k=[a b c d]';
    
    while(i<=3)
        %Magnetometer compensation
        m=Magn/norm(Magn);
        q_coniug=[q(1,1); -q(2:4,1)];
        hTemp=QuaternionProduct(q,[0;m]);
        h=QuaternionProduct(hTemp,q_coniug);
        bMagn=[sqrt(h(2,1)^2+h(3,1)^2) 0 h(4,1)]';
        bMagn=bMagn/norm(bMagn);
        %End magnetometer compensation
        
        J_nk=ComputeJacobian(a,b,c,d,Acc(1,1),Acc(2,1),Acc(3,1),Magn(1,1),Magn(2,1),Magn(3,1));

        M=ComputeM_Matrix(a,b,c,d);

        y_e=[[0 0 1]';bMagn];

        y_b=[Acc;Magn];

        %Gauss-Newton step
        n=n_k-((J_nk'*J_nk)^-1)*J_nk'*(y_e-M*y_b);
        n=n/norm(n);
        a=n(1,1);
        b=n(2,1);
        c=n(3,1);
        d=n(4,1);
        n_k=n;
        q=[d a b c]';
        
        i=i+1;
    end
    
    n=[n(4,1);n(1:3,1)];
end

