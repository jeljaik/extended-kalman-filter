psswd = perms(1:4);

psswdCell = cell(size(perms(1:4)));
idx_m = find(psswd == 1);
idx_l = find(psswd == 2);
idx_N = find(psswd == 3);
idx_R = find(psswd == 4);
psswdCell(idx_m) = 
for i=idx_m
    psswdCell{i} = 'm';
end
for i=idx_l(1):idx_l(end)
    psswdCell{i} = 'l';
end
for i=idx_N(1):idx_N(end)
    psswdCell{i} = 'N';
end
for i=idx_R(1):idx_R(end)
    psswdCell{i} = 'R';
end
