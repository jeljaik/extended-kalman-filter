function plotAndSaveHumanoidsFigs(plotFigBaseFolder)

%This function is used to compare various plots for different experiments.
% Note the sequence of input vectors which reflects what you want to
% compare. Also beware that all the input vectors except variable 'compare' are of the same length.
% experiment - 'leg', 'foot'
% whichDataset - 'old', 'new'
% trialNum - 1,2,3,4 (new), 7(old)
% expID - 1,2 (foot), 1,2,3,4,5(leg)
% processType - 'withoutCompliance','dualState'
% measurementType - 'withoutSkin','withSkin',,'dualState','withoutlegFT','dualStateWithoutlegFT'
% plotDivide - 'true','false' : true gives steady state and dynamic state
% behavior individually
% legends - as a vector of same length
% compare - 'orientation','orientationvariance', 'stiffness' , 'FRI' , 'all'


% comparePlots(experiment,whichDataset,trialNum,expID,processType,measurementType,plotDivide,legends)
experiment = cellstr(['leg';'leg';'leg']);
whichDataset = cellstr(['old';'old';'old']);
trialNum = [7;7;7];
expID = [1;2;3];
processType = cellstr(['withoutCompliance';'withoutCompliance';'dualState        ']);
measurementType = cellstr(['withoutSkin';'withSkin   ';'dualState  ']);
plotDivide = cellstr(['false';'false';'false']);
legends = cellstr(['NSNC';'WSNC';'WSWC']);
compare = 'orientation'; 


comparePlots(experiment,whichDataset,trialNum,expID,processType,measurementType,plotDivide,legends,compare);

% compare = 'orientationvariance'; 
% comparePlots(experiment,whichDataset,trialNum,expID,processType,measurementType,plotDivide,legends,compare);   
    
    
    if(~exist(plotFigBaseFolder,'dir'))
        mkdir(plotFigBaseFolder);
    end

    plotFigBaseName = strcat('./',plotFigBaseFolder,'predicted');

%     selectedFigList = [1,2,3,4,6];
    selectedFigList = [1,2];
    FigName = { 'OrientationEstimatesComparisonLeg',...
                'OrientationVarianceComparisonLeg',...
                'OrientationEstimatesComparisonFoot',...
                'EstimatedFRIComparisonFoot',...
                'OrientationEstiamteSurfaceComparisonFoot',...
                'StiffnessNormEvolutionComparison'};


    for i = 1:length(selectedFigList)
         figure(selectedFigList(i))
         set(gca,'FontSize',12);
         set(gcf,'Renderer','OpenGL');
         print('-dpdf','-r200',strcat(plotFigBaseName,FigName{i}),'-opengl');
    end
end