function [ F ] = shotRotatingPlot(  )
%SHOTROTATINGPLOT 
    
    N_FRAME = 20;
    fileName = 'arm.avi';
    
    % Preallocate the struct array for the struct returned by getframe
    F(N_FRAME) = struct('cdata',[],'colormap',[]);
    % Prepare the new file.
    vidObj = VideoWriter(fileName);
    open(vidObj);
    
    % Record the movie
    for j = 1:N_FRAME 
        view([j*10 30]);
        F(j) = getframe;
        writeVideo(vidObj,F(j));
    end
    movie(F,2);    
    close(vidObj);

end

