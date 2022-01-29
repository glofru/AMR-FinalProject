classdef State < handle
    % Class to keep and work with a state
    %

    properties
        x
        y
        
        state
        cost
        
        g
        rhs
        
        k
    end

    methods
        function obj = State(x, y, state, cost)
            arguments
                %
                x %{}
                %
                y %{}
                %
                state {} = MapState.UNKNOWN
                %
                cost = 1
            end
            obj.x = x;
            obj.y = y;
            obj.state = state;
            obj.cost = cost;
            
            obj.g = 0;
            obj.rhs = 0;
            
            obj.k = 0;
        end

        function K = calcKey(obj, Sstart)
            k1 = min(obj.g, obj.rhs + obj.h(Sstart));
            k2 = min(obj.g, obj.rhs);

            K = [k1, k2];
        end
        
        function res = h(obj, s)
            res = norm([obj.x - s.x, obj.y - s.y]);
        end

        function res = c(obj, s)
            res = obj.cost * norm([obj.x - s.x, obj.y - s.y]);
        end

        function e = eq(obj, s)
            e = (obj.x == s.x && obj.y == s.y);
        end
    end
end