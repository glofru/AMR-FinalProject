function res = min2(v1, v2)
    if (v1(1) < v2(1))
        res = true;
    elseif (v1(1) == v2(1))
        if (v1(2) < v2(2))
            res = true;
        elseif (v1(2) == v2(2))
            res = false;
        else % (v1(2) > v2(2))
            res = false;
        end
    else % (v1(1) > v2(1))
        res = false;
    end
end