classdef State < handle
    properties (SetAccess = private)
        x
        y
    end
    
    properties
        parent
        state
        tag
        
        h
        k
    end

    methods
        function obj = State(x, y)
            arguments
                x {}
                y {}
            end
            obj.x = x;
            obj.y = y;
            obj.parent = State.empty;
            obj.state = MapState.EMPTY;
            obj.tag = StateTag.NEW;
            
            obj.h = 0;
            obj.k = 0;
        end

        function res = cost(obj, state)
            if obj.state == MapState.OBSTACLE || state.state == MapState.OBSTACLE
                res = Inf;
            else
                res = sqrt((obj.x - state.x)^2 + (obj.y - state.y)^2);
            end
        end

        function e = eq(obj, s)
            e = obj.x == s.x && obj.y == s.y;
        end
    end
end