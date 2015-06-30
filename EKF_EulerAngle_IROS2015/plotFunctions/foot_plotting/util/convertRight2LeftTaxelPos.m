%% FOREARM 
%rotation matrix from left to right wrist
R_l2r       = eye(3);
R_l2r(2,2)  = -1;
R_l2r(3,3)  = -1;

pos_r = forearmTaxels;
pos_l = (R_l2r * pos_r')';

%% UPPER ARM
% rotation matrix from left to right elbow
Ru_r2l       = eye(3);
Ru_r2l(2,2)  = -1;
Ru_r2l(1,1)  = -1;

posu_l = upperArmTaxels;
posu_r = (Ru_r2l * posu_l')';

% the internal and external patches have to be swapped
% the lower patch has to be mirrored
upArmLow = upperArmTaxels;
upArmLow(:,3) = -upArmLow(:,3);
subplot(2,2,1);
plotSkinArm(1, struct('valid',[],'pos',[]),upArmLow(:,1:3),[0 0],1,[],[],1);

% compute y mean
validUpArmLow = zeros(1,3);
j=1;
for i=1:size(upArmLow,1)
    if(~(upArmLow(i,1)==0 && upArmLow(i,2)==0 && upArmLow(i,3)==0))
        validUpArmLow(j,1:3) = upArmLow(i,1:3);
        j=j+1;
    end
end
yMean = mean(validUpArmLow(:,2));
% shift of ymean
upArmLow(:,2) = upArmLow(:,2)-yMean;
subplot(2,2,2);
plotSkinArm(1, struct('valid',[],'pos',[]),upArmLow(:,1:3),[0 0],0,[],[],1);
% mirror the y
upArmLow(:,2) = -upArmLow(:,2);
subplot(2,2,3);
plotSkinArm(1, struct('valid',[],'pos',[]),upArmLow(:,1:3),[0 0],0,[],[],1);
% shift back of ymean
upArmLow(:,2) = upArmLow(:,2)+yMean;
subplot(2,2,4);
plotSkinArm(1, struct('valid',[],'pos',[]),upArmLow(:,1:3),[0 0],0,[],[],1);