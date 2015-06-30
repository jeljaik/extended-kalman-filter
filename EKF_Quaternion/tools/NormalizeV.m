function [normalized] = NormalizeV(aVector)
    normalized = aVector./norm(aVector);
end