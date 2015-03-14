function plotAndSaveFigs(dataBaseFolder,plotFigBaseFolder)
    plotResultsOutput_withSkin(dataBaseFolder);

    %plotFigBaseFolder = 'plots/irosMain/';
    %dataBaseFolder = './data/acclTests/';
    if(~exist(plotFigBaseFolder))
        mkdir(plotFigBaseFolder);
    end

    plotFigBaseName = strcat('./',plotFigBaseFolder,'predicted');

    selectedFigList = [1,2,3,4,6,7];
    FigName = {'UpperWrench',...
                'LowerWrench',...
                'Velocities',...
                'Orientation',...
                'FRI_timeSeries',...
                'FRI_trajectory'};


    for i = 1:length(selectedFigList)
         figure(selectedFigList(i))
         set(gca,'FontSize',12);
         set(gcf,'Renderer','OpenGL');
         print('-depsc2','-r200',strcat(plotFigBaseName,FigName{i}),'-opengl');
    end
end