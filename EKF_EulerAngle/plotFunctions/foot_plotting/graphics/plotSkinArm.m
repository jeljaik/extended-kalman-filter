function [ ] = plotSkinArm( rightArm, forearmTaxels, upperArmTaxels, ...
    joints, plotLabel, errors, refTaxels, plotMesh )
%PLOTSKINARM Plot the taxel positions of the whole arm, together with the
%cover meshes.
%
% INPUT:
% -rightArm: 1 if right arm, 0 if left arm
% -forearmTaxels: struct containing 2 arrays:
%   - valid: nx1 matrix, ~0 if the related taxel position is valid, 0 otherwise
%   - pos: nx3 matrix with positions of the forearm taxels w.r.t.
%           <8> (wrist)
% -upperArmTaxels: mx3 matrix with positions of the upper arm taxels w.r.t.
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

%     if(rightArm==0)
        %% Project the upperarm taxel in the wrist reference frame <8>
        theta3 = joints(1)*pi/180;
        theta4 = joints(2)*pi/180;
        if(rightArm)
            H_67=evalDHMatrix(   0.015,       0,     pi/2,    theta3); 
            H_78=evalDHMatrix(    0,  -0.1373,     pi/2,    theta4-pi/2);
        else
            H_67=evalDHMatrix( -0.015,      0,     pi/2,  theta3); 
            H_78=evalDHMatrix(   0,   0.1373,     pi/2, theta4-pi/2); 
        end
        H_86 = Hinv(H_67*H_78);
        
        if(size(upperArmTaxels,1)>0)
            % if the points are not represented in homogeneous coordinates 
            % add the fourth column
            upperArmTaxels(:,4) = 1;
            upperArmTaxels = (H_86 * upperArmTaxels')';
            upperArmTaxels = upperArmTaxels(:,1:3);
        end

        %% read the upperarm cover mesh and project it in the wrist ref frame
        %% <8>
        [VupArm, FupArm] = read_plot_upperarm_mesh(rightArm, 0);
        VupArm(:,4) = 1;
        VupArm = (H_86 * VupArm')';
        VupArm = VupArm(:,1:3);
        %fprintf('Upper arm mesh vertecis: %d\n', size(VupArm,1));
%     end
    
    %% read the forearm cover mesh
    [VforeArm, FforeArm] = read_plot_forearm_mesh(rightArm, 0);
    hold all;
    
    
    % Now all the points are expressed w.r.t. the wrist ref frame <8>

    %% Plot the forearm taxels
    n = size(forearmTaxels.pos,1);
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
            if(forearmTaxels.valid(i))
                plot3(forearmTaxels.pos(i,1),forearmTaxels.pos(i,2),forearmTaxels.pos(i,3),...
                    'ok','markerfacecolor',colorSet(i,:));             
                if(plotLabel)
                    text(forearmTaxels.pos(i,1),forearmTaxels.pos(i,2),forearmTaxels.pos(i,3),...
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
                if(forearmTaxels.valid(1+i*12))
                    t = i*12;
                    corners = refTaxels([t+2, t+6, t+10, t+2],:);
                    plot3(corners(:,1),corners(:,2),corners(:,3),...
                        'color',colorSet(1+t,:), 'LineWidth', 2);
                end
            end
                    
        end
    end

    %% plot the forearm mesh
    if(plotMesh)
        trisurf(FforeArm,VforeArm(:,1),VforeArm(:,2),VforeArm(:,3),'facealpha', 0.1,'FaceColor',[0.26,0.33,1.0 ]);
    end
    hold all;
        
    if(size(upperArmTaxels,1)>0)
        %% plot the upper arm taxels
        triangles = ceil(size(upperArmTaxels,1)/12);
        fprintf('Upperarm taxels: %d, triangles: %d\n', size(upperArmTaxels,1), triangles);
        colorSet = varycolor(ceil(triangles/4));
        colorSet = colorSet(randperm(ceil(triangles/4)),:);   % shuffle the colors
        for i=0:(triangles-1)
            t = 1+i*12;     
            plot3(upperArmTaxels(t:t+11,1),upperArmTaxels(t:t+11,2),upperArmTaxels(t:t+11,3),'ok', ...
                'markerfacecolor',colorSet(mod(i,size(colorSet,1))+1,:));
    %       scatter3(upperArmTaxels(t:t+11,1),upperArmTaxels(t:t+11,2),upperArmTaxels(t:t+11,3),50,colorSet(mod(i,triangles/4)+1,:),'filled');
%             if(plotLabel)
%                 text(upperArmTaxels(t:t+11,1),upperArmTaxels(t:t+11,2),upperArmTaxels(t:t+11,3),labels, 'Fontsize', 20,'Fontweight','Bold');
%             end
            hold all;
        end    
    
        
    end
    %% plot the upper arm mesh
    if(plotMesh)
        trisurf(FupArm,VupArm(:,1),VupArm(:,2),VupArm(:,3),'facealpha', 0.1,'FaceColor',[0.26,0.33,1.0 ]);
    end
    axis equal;
%     xlabel('X');
%     ylabel('Y');
%     zlabel('Z');
    if(rightArm)
        if(size(upperArmTaxels,1)>-1)
            set(gca,'YLim',[0.0 0.27]);
        else
            set(gca,'YLim',[0.0 0.12]);
        end
    else
        %set(gca,'YLim',[-0.27 0.0]);
    end
    %set(gca,'XLim',[-0.05 0.05]);
end
