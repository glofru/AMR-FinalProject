classdef State < handle
    properties (Constant)
        
    end
    
    properties
        x
        y
        % parent
        state
        
        g
        rhs
        
        k
    end

    methods
        function obj = State(x, y, state)
            arguments
                %
                x {}
                %
                y {}
                %
                state {} = MapState.UNKNOWN
            end
            obj.x = x;
            obj.y = y;
            obj.state = state;
            
            obj.g = 0;
            obj.rhs = 0;
            
            obj.k = 0;
        end

        function K = calcKey(obj, Sstart, km)
            arguments
                obj
                
                Sstart
                
                km = 0
            end
            k1 = min(obj.g, obj.rhs + obj.h(Sstart) + km);
            k2 = min(obj.g, obj.rhs);

            K = [k1, k2];
        end
        
        function res = h(obj, s)
            res = norm([obj.x - s.x, obj.y - s.y]);
        end

        function res = c(obj, state)
            res = 1;
        end

        function e = eq(obj, s)
            e = (obj.x == s.x && obj.y == s.y);
        end
    end
end