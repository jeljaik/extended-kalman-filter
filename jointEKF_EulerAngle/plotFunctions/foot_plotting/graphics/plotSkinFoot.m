function [ ] = plotSkinFoot( rightFoot, footTaxels, ...
    joints, plotLabel, errors, refTaxels, plotMesh )
%PLOTSKINFoot Plot the taxel positions of the whole Foot, together with the
%cover meshes.
%
% INPUT:
% -rightFoot: 1 if right Foot, 0 if left Foot
% -foreFootTaxels: struct containing 2 arrays:
%   - valid: nx1 matrix, ~0 if the related taxel position is valid, 0 otherwise
%   - pos: nx3 matrix with positions of the foreFoot taxels w.r.t.
%           <8> (wrist)
% -upperFootTaxels: mx3 matrix with positions of the upper Foot taxels w.r.t.
% <6> (elbow)
% -joints: joint 3 and 4 angles, in degrees
% -plotLabel: true if the taxel numbers have to be plotted
% -errors: array containing the error (distance) of every taxel, that is
% used for selecting the color of that taxel
    if(nargin<6 || isempty(errors))
        COLOR_ERROR = 0;
    else
        COLOR_ERROR = 1;
    end
    
    if(nargin<7 || isempty(refTaxels))
        PLOT_REF_TAX = 0;
    else
        PLOT_REF_TAX = 1;
    end
    
    if(nargin<8)
        plotMesh = 1;
    end

%     if(rightFoot==0)


        %% read the upperFoot cover mesh and project it in the wrist ref frame
        %% <8>
        [VupFoot, FupFoot] = read_plot_foot_mesh(rightFoot, 0);
        VupFoot(:,4) = 1;
        VupFoot = (VupFoot')';
        VupFoot = VupFoot(:,1:3);
        %fprintf('Upper Foot mesh vertecis: %d\n', size(VupFoot,1));
%     end
    
    %% read the foreFoot cover mesh
    [Vfoot, Ffoot] = read_plot_foot_mesh(rightFoot, 0);
    hold all;
    
    
    % Now all the points are expressed w.r.t. the wrist ref frame <8>

    %% Plot the foreFoot taxels
    n = size(footTaxels.pos,1);
    if(n>0)
        if(COLOR_ERROR)
            errorsNorm = errors/max(errors);
            cmap = colormap;
            colorSet = cmap(1+round(errorsNorm*(size(cmap,1)-1)), :);
            colorbar;
        else
            % plot every triangle with a different color
            triangles = ceil(n/12);
            colorSet2 = varycolor(triangles/2);
            colorSet2 = colorSet2(randperm(triangles/2),:);   % shuffle the colors
            colorSet = zeros(n, 3);
            for i=0:triangles
                colorSet(1+i*12:i*12+12,:) = repmat(colorSet2(mod(i,triangles/2)+1,:), 12, 1);
            end
        end

        for i=1:n
            if(footTaxels.valid(i))
                plot3(footTaxels.pos(i,1),footTaxels.pos(i,2),footTaxels.pos(i,3),...
                    'ok','markerfacecolor',colorSet(i,:));             
                if(plotLabel)
                    text(footTaxels.pos(i,1),footTaxels.pos(i,2),footTaxels.pos(i,3),...
                        num2str(mod(i-1,12)+1), 'Fontsize', 20,'Fontweight','Bold');
                end                
                hold all;
            end
        end
        
        if(PLOT_REF_TAX)
            % plot the triangle edges (taxels 2, 6 and 10 are in the
            % corners)
            triangles = ceil(n/12);
            for i=0:triangles-1
                if(footTaxels.valid(1+i*12))
                    t = i*12;
                    corners = refTaxels([t+2, t+6, t+10, t+2],:);
                    plot3(corners(:,1),corners(:,2),corners(:,3),...
                        'color',colorSet(1+t,:), 'LineWidth', 2);
                end
            end
                    
        end
    end

    %% plot the foreFoot mesh
    if(plotMesh)
        trisurf(Ffoot,Vfoot(:,1),Vfoot(:,2),Vfoot(:,3),'facealpha', 0.1,'FaceColor',[0.26,0.33,1.0 ]);
    end
    hold all;
end
