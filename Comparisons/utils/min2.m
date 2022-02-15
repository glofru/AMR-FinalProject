% check if v1 is < than v2, for v1 and v2 as 2-dim vectors
function res = min2(v1, v2)
    if v1(1) < v2(1) || (v1(1) == v2(1) && v1(2) < v2(2))
        res = true;
    else
        res = false;
    end
end


