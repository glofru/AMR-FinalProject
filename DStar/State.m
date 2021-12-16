classdef State < handle
    properties
        x
        y
        parent
        state
        tag
        h
        k
    end

    methods
        function obj = State(x, y)
            obj.x = x;
            obj.y = y;
            obj.state = ".";
            obj.tag = "new";
            obj.h = 0;
            obj.k = 0;
        end

        function c = cost(obj, state)
            if obj.state == "#" || state.state == "#"
                c = Inf;
            else
                c = sqrt((obj.x - state.x)^2 + (obj.y - state.y)^2);
            end
        end

        function e = eq(a, b)
            e = a.x == b.x && a.y == b.y;
        end
    end
end