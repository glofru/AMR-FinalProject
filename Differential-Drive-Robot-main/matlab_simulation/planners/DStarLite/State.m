classdef State < handle
    properties (Constant)
        
    end
    
    properties
        x
        y
        parent
        state
        tag
        
        g
        h
        rhs
        
        k
    end

    methods
        function obj = State(x, y, state)
            arguments
                x {}
                y {}

                state {} = MapState.UNKNOWN
            end
            obj.x = x;
            obj.y = y;
            obj.parent = State.empty;
            obj.state = state;
            obj.tag = StateTag.NEW;
            
            obj.g = 0;
            obj.h = 0;
            obj.rhs = 0;
            
            obj.k = 0;
        end

        function K = calcKey(obj, Sstart, km)
            arguments
                obj
                
                Sstart
                
                km = 0
            end
            h_ = norm([obj.x - Sstart.x, obj.y - Sstart.y]);
            k1 = min(obj.g, obj.rhs + h_ + km);
            k2 = min(obj.g, obj.rhs);

            K = [k1, k2];
        end

        function res = c(obj, state)
            res = 1;
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