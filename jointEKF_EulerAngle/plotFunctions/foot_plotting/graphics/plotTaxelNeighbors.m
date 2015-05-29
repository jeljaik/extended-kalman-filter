function [ ] = plotTaxelNeighbors( taxels, neighbors, valid )
%PLOTTAXELNEIGHBORS Plot the taxels neighbor relations as lines.
% - taxels: matrix Nx3 containing the 3d positions of N taxels
% - neighbors: NxN matrix where the (i,j)th element is 0 iff taxel i and
%               taxel j are not neighbors, ~0 otherwise
% - valid: N vector, 0 if the taxel is not valid, ~0 otherwise

    for i=1:size(neighbors,1)
        for j=i+1:size(neighbors,1)
            if(valid(i) && valid(j) && neighbors(i,j)~=0)
                line([taxels(i,1) taxels(j,1)], [taxels(i,2) ...
                    taxels(j,2)], [taxels(i,3) taxels(j,3)]);
            end
        end
    end
end

