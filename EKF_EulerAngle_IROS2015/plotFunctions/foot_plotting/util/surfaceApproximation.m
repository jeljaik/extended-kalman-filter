function [ ] = surfaceApproximation( )
%SURFACEAPPROXIMATION Given a list of 3d points find the surface that best
%interpolates them
    rightArm = 1;
    doPlotSkin = 1;
    plotLinSurface = 1;
    plotQuadSurface = 0;
    plotCubSurface = 0;

    [V, ] = read_plot_forearm_mesh(rightArm, doPlotSkin);
    load('../data/right_forearm/lower/refTaxelsRot.mat');
    taxels = filterRefTaxels(refTaxels);
    hold on;
    plot3(taxels(1,1),taxels(1,2),taxels(1,3), 'ok');
    %plot3(taxels(:,1),taxels(:,2),taxels(:,3), 'ok');
    
    
    %% linear surface
    if(plotLinSurface)
        X = [ones(size(taxels,1),1) taxels(:,1:2)];
        Y = taxels(:,3);
        B = X\Y;
        
        [X,Y] = meshgrid(-0.04:.01:0.04, 0:.01:0.13);
        Z = B(3).*Y + B(2).*X + B(1);
        surf(X,Y,Z);
    end
    
    %% quadratic surface
    if(plotQuadSurface)
        X = [ones(size(taxels,1),1) taxels(:,1:2) taxels(:,1:2).^2];
        Y = taxels(:,3);
        B = X\Y;
        
        [X,Y] = meshgrid(-0.04:.01:0.04, 0:.01:0.13);
        Z = B(5).*Y.^2 + B(4).*X.^2 + B(3).*Y + B(2).*X + B(1);
        surf(X,Y,Z);
    end
    
    %% cubic surface
    if(plotCubSurface)
        X = [ones(size(taxels,1),1) taxels(:,1:2) taxels(:,1:2).^2 taxels(:,1:2).^3];
        Y = taxels(:,3);
        B = X\Y;
        
        [X,Y] = meshgrid(-0.04:.01:0.04, 0:.01:0.13);
        Z = B(7).*Y.^3 + B(6).*X.^3 + B(5).*Y.^2 + B(4).*X.^2 + B(3).*Y + B(2).*X + B(1);
        surf(X,Y,Z);                
    end

end

function [taxelFilt] = filterRefTaxels(taxels)
    taxelFilt = zeros(0,3);
    for i=1:size(taxels,1)
        if(taxels(i,1)~=0 || taxels(i,2)~=0 || taxels(i,3)~=0)
            taxelFilt = [taxelFilt; taxels(i,:)];
        end
    end
end

function [dist] = computeDistance(B, P)
    % compute the distance between a polinomial surface and a 3D point
    % B is the vector containing the coefficients of the polinomial surface
    % B(1) + B(2)*x + B(3)*y + B(4)*x^2 + B(5)*y^2 + ...
    % P is the 3D point
    
    PX = P(1); PY = P(2); PZ = P(3);    
    % compute the gradient
    dF = zeros(2,1);
    for i=1:(length(B)-1)/2
        dF(1) = dF(1) + B(2*i)*PX^(i-1);
        dF(2) = dF(2) + B(2*i+1)*PY^(i-1);
    end
    k = dF'*[PX;PY] - PZ;
    Z = dF(1).*X + dF(2).*Y - k;
    surf(X,Y,Z);

    normal = [dF; -1];
    lambda = 0.1;
    P1 = P - lambda*normal;
    P2 = P + lambda*normal;
    plot3([P1(1) P2(1)], [P1(2) P2(2)], [P1(3) P2(3)]);
    
    dist = 0;
end