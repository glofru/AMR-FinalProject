classdef StateD < handle
    properties (Constant)
        TAG_NEW = "new";
        TAG_OPEN = "open";
        TAG_CLOSED = "closed";
    end
    
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
            obj.parent = State.empty;
            obj.state = Map.MAP_EMPTY;
            obj.tag = State.TAG_NEW;
            obj.h = 0;
            obj.k = 0;
        end

        function c = cost(obj, state)
            if obj.state == Map.MAP_OBSTACLE || state.state == Map.MAP_OBSTACLE
                c = Inf;
            else
                c = sqrt((obj.x - state.x)^2 + (obj.y - state.y)^2);
            end
        end

        function e = eq(a, b)
            arguments
                a = 0
                b = 0
            end
            switch nargin
                case 2
                    % we have all the parameters, fine
                otherwise
                    e = 0;
                return
            end
            if isempty(a) || isempty(b)
                e = 0;
                return
            end
            
            e = (a.x == b.x && a.y == b.y);
        end
    end
end