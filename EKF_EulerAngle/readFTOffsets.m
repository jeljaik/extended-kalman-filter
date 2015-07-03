function [FT2,FT3,FT4,FT5] = readFTOffsets(expPath)


% reads the leg FTsensor offsets from the file - set header lines option to 3 to
% skip the first three lines which includes arm FT offsets. Store the data
% column wise.

[d d d d d sensor d ft1 ft2 ft3 ft4 ft5 ft6] = textread(strcat(expPath,'ftSensorsOffset.txt'),'%s %s %s %s %s %s %s %f %f %f %f %f %f','headerlines',3);

%Left leg
FT2 = [ft1(1) ft2(1) ft3(1) ft4(1) ft5(1) ft6(1)];
%Left foot
FT3 = [ft1(2) ft2(2) ft3(2) ft4(2) ft5(2) ft6(2)];
%Right leg
FT4 = [ft1(3) ft2(3) ft3(3) ft4(3) ft5(3) ft6(3)];
%Right foot
FT5 = [ft1(4) ft2(4) ft3(4) ft4(4) ft5(4) ft6(4)];

end