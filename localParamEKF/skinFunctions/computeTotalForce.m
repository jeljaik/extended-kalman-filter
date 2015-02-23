function [totalForces] = computeTotalForce(processedSkinData, whichStiffVec)
% COMPUTETOTALFORCE(processedSkinData, whichStiffVec)
% @processedSkinData: Processed skin data without package id or time stamp
% @whichStiffVec: Stiffness vector estimated using 'normalForces' only or
% 'forceTorques' (force and torques).

if(strcmp(whichStiffVec,'normalForces'))
    w_vec = load('footStiffnessMatrix.mat','best_w');
    w_vec = w_vec.best_w;
    totalForces = processedSkinData*w_vec;
else
    if(strcmp(whichStiffVec,'forceTorques'))
        display('ERROR: option still not available');
    end
end

end