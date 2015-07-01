function [totalForce] = computeTotalForce(processedSkinData, whichStiffVec)
% COMPUTETOTALFORCE(processedSkinData, whichStiffVec)
% @processedSkinData: Processed skin data without package id or time stamp
% @whichStiffVec: Stiffness vector estimated using 'normalForces' only or
% 'forceTorques' (force and torques).

if(strcmp(whichStiffVec,'normalForces'))
    %% Modified by JORHABIB on 24/6/2015 to include new best_w_padded from estimatedStiffVecAndActiveTaxels.mat
    % The last .mat file was created by running test_on_large_data.m in
    % ~/Software/skin where the variable taxelsPos comes from the CAD
    % extracted taxel positions by Prashanth.
    w_vec = load('estimatedStiffVecAndActiveTaxels.mat','best_w_padded');
%     w_vec = load('best_w_padded.mat','best_w_padded');
    w_vec = w_vec.best_w_padded;
    % The following totalForces corresponds to the total normal force as
    % measured by the FT sensor
    totalForce = processedSkinData*w_vec;
else
    if(strcmp(whichStiffVec,'forceTorques'))
        display('ERROR: option still not available');
    end
end

end