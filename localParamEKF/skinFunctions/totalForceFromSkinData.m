function totalForce = totalForceFromSkinData(typeExperiment, numberExperiment, foot)
% TOTALFORCEFROMSKINDATA(typeExperiment, numberExperiment, foot)
% @typeExperiment: 'backwardTipping' or 'forwardTipping'
% @numberExperiment: Integer from 1 to 6 (for now);
% @foot: 'left' or 'right'

rawSkinDataDumper = load(['../robotData/' typeExperiment '/dumperTippingSetup0' num2str(numberExperiment) '/icub/skin/' foot '_foot/data.log']);
processedSkinData = dataPostProcessing(rawSkinDataDumper, 'normalForces');
totalForce = computeTotalForce(processedSkinData, 'normalForces');

end