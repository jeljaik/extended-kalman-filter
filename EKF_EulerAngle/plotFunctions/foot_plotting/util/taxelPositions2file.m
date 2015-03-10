function [ ] = taxelPositions2file( fileName, taxelPos, addFakeNormals )
%TAXELPOSITIONS2FILE Write the taxel positions to text file
% This is used to create a text file that can be read by a C++ application
% - fileName: the name of the text file to write
% - taxelPos: matrix Nx3 containing the 3d position of the taxels
% - addFakeNormal: if ~0 then 3 zero columns are added to taxelPos

    if(nargin<3)
        addFakeNormals=0;
    end
    
    if(addFakeNormals)
        taxelPos = [taxelPos zeros(size(taxelPos))];
    end

    dlmwrite(fileName, taxelPos, 'delimiter', ' ', 'precision', '% .3f');

end

