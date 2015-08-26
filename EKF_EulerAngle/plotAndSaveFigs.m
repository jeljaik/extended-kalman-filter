 function plotAndSaveFigs(dataBaseFolder,plotFigBaseFolder,filter)
    plotResultsOutput_withSkin(dataBaseFolder,filter);

    %plotFigBaseFolder = 'plots/irosMain/';
    %dataBaseFolder = './data/acclTests/';
    if(~exist(plotFigBaseFolder,'dir'))
        mkdir(plotFigBaseFolder);
    end

    plotFigBaseName = strcat('./',plotFigBaseFolder,'predicted');
    
    if(strcmp(filter,'ekf') == 1)
    selectedFigList = [1,2,3,4];
    FigName = {'UpperWrench',...
                'LowerWrench',...
                'Velocities',...
                'Orientation'};
    else if(strcmp(filter,'jekf') == 1 || strcmp(filter,'dekf') == 1)
    
    selectedFigList = [1,2,3,4,5];
    FigName = {'UpperWrench',...
                'LowerWrench',...
                'Velocities',...
                'Orientation',...
                'Stiffness and Damping'};
        end
    end
            
    for i = 1:length(selectedFigList)
         figure(selectedFigList(i))
         set(gca,'FontSize',12);
         set(gcf,'Renderer','OpenGL');
         print('-djpeg','-r200',strcat(plotFigBaseName,FigName{i}),'-opengl');
    end
end