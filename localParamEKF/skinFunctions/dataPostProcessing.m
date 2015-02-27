function [processedSkinData] = dataPostProcessing(rawSkinDataShort, whichStiffVec, threshold)
% DATAPOSTPROCESSING(rawSkinData, whichStiffVec, threshold)
% @param  rawSkinData [n x 386]: Raw skin data as retrieved by the
% dataDumper where each row corresponds to a sample reading.
% 
% @param whichStiffVec: Options are 'normalForces', 'forceTorque'.
% 'normalForces': Based on Luca&Carlo's first way of performing the Least
% Squares Estimation in which, assuming CK=b, C is simply the [n x 384]
% matrix with the processed skin compression data, K the [384 x 1]
% vector and b the [n x 1] normal forces.
% 'forceTorque': Based on Luca&Carlo's second way of performing the Least
% Squares Estimation in which normal forces and torques were also
% considered. The description of the procedure is the one presented in the
% paper.
% 
% @param threshold: Skin activation threshold (UNUSED)
%
% @output skinProcessedData [n x 384]: Filtered and processed skin data. Ready for
% multiplication with the stiffness vector.
% 
%
% NOTES ON rawSkinData:
% It is assumed that the raw skin data has been acquired via the yarpdatadumper
% whose documentation can be found in http://wiki.icub.org/yarpdoc/group__yarpdatadumper.html
% Thus, as stated in the documentation, the raw skin data samples have the
% following structure:
% [pck id] [time stamp] [bottle content (or image_file_name)]
% In our case, we have for the feet skin soles 384 taxels, thus making our
% samples of size [n x (2 + 384)], where the 2 corresponds to the package
% ID and their corresponding time stamp. 
%
% NOTES ON processedSkinData:
% This method does exactly what has been done in the processing of raw skin
% data by `load_n_filter.m` and `get_input_output.m`. Be it correct or not,
% since the estimation of the stiffness matrix was assessed with test data
% that was processed the same way, it makes just sense to do the same for
% our data. 
% 
% POSSIBLE IMPROVEMENTS: 
% Instead of recording readings from /icub/skin/right_foot or /left_foot it
% might be better to do it from the compensated ports, i.e.
% /icub/skin/right_foot/comp

if (nargin < 3)
    % default threshold to filter false positives
    threshold = 50;
end

if (nargin < 2)
    whichStiffVec='normalForces';
end

% rawSkinDataShort  = rawSkinData(:, 3:end);
processedSkinData = 256-rawSkinDataShort;
processedSkinData = processedSkinData/255;

if (strcmp(whichStiffVec,'forceTorque'))
    % Retrieve taxel positions
    taxelPos = load('Tmatrix.mat');
    taxelPos = taxelPos.ans;
end



end