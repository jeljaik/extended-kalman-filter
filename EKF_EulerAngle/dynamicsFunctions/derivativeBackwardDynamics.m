function A = derivativeBackwardDynamics(Xhat, model)

dt = model.dt;
% 
% df = rigidBodyDynamicsDerivatives(...
%     Xhat(1) ,Xhat(2) ,Xhat(3),...
%     Xhat(4) ,Xhat(5) ,Xhat(6),...
%     Xhat(7) ,Xhat(8) ,Xhat(9),...
%     Xhat(10),Xhat(11),Xhat(12),...
%     Xhat(13),Xhat(14),Xhat(15),Xhat(16),...
%     model.I(1,1),model.I(2,2),model.I(3,3),model.m,model.g);


df = rigidBodyDynamicsDerivatives(...
    Xhat(1) ,Xhat(2) ,Xhat(3),...
    Xhat(4) ,Xhat(5) ,Xhat(6),...
    Xhat(7) ,Xhat(8) ,Xhat(9),...
    Xhat(10),Xhat(11),Xhat(12),...
    Xhat(13),Xhat(14),Xhat(15),...
    Xhat(16),Xhat(17),Xhat(18),...
    Xhat(19),Xhat(20),Xhat(21),...
    model.I(1,1), model.I(1,2),model.I(1,3),model.I(2,2),model.I(2,3),model.I(3,3),model.m,model.g,model.gRot(1),model.gRot(2),model.gRot(3));



A = inv(eye(size(df)) - df.*dt);