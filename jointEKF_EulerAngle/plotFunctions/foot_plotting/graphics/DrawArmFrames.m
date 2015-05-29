%% Draw the cover meshes, the wrist ref frame and the F/T sensor ref frame
joints = [45 0];
rightArm = 1;
clf;
plotSkinArm(rightArm, struct('valid',[],'pos',[]), zeros(0,3), joints, 0);
[ ~, H_8S ] = ftSensor2wristTransformation( rightArm, [], joints(1), joints(2) );
[ ~, H_6S ] = ftSensor2elbowTransformation( rightArm, []);
H_86 = H_8S * Hinv(H_6S);

ylabel('y');
set(gca,'ZLim',[-0.05 0.12]);

%DrawRefFrame(eye(4), 'wrist');
%DrawRefFrame(H_8S, 'sensor');
%DrawRefFrame(H_86, 'elbow');