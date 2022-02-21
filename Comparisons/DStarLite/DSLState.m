classdef DSLState < handle
    %
    %
    
    properties(Constant) % enumeration
        OBSTACLE = "█";
        UNKNOWN = "▓";
        EMPTY = "░";

        START = "ⓢ";
        GOAL = "♛";
        POSITION = "☺";

        VISITED = "╬";
        PATH = "≡";
    end
    
    properties
        % x coord
        x
        % y coord
        y
        
        % state of this cell
        state
        % cost of a step
        cost
        
        % g-value
        g
        % rhs-value
        rhs
        
        % key pair
        k
    end

    methods
        % DSLState constructor
        function obj = DSLState()
            obj.state = DSLState.UNKNOWN;
            obj.g = Inf;
            obj.rhs = Inf;
        end
        
        function setPos(obj, x, y, cost)
            obj.x = x;
            obj.y = y;
            obj.cost = cost;
        end
        
        
        % return the key pair
        function K = calcKey(obj, Sstart, km)
            arguments
                % this state
                obj
                % current state
                Sstart
                % km parameter
                km = 0
            end
            k1 = min(obj.g, obj.rhs + obj.h(Sstart) + km);
            k2 = min(obj.g, obj.rhs);

            K = [k1, k2];
        end
        
        % return the heuristic from obj to s
        function res = h(obj, s)
            res = norm([obj.x - s.x, obj.y - s.y]);
        end
        
        % return the cost from obj to s
        function res = c(obj, s)
            res = obj.cost * norm([obj.x - s.x, obj.y - s.y]);
        end
        
        
        % check if 2 states are equal
        function e = eq(obj, s)
            e = (obj.x == s.x && obj.y == s.y);
        end
        
        
        % return the color of the state
        function color = getColor(obj)
            switch obj.state
                case DSLState.OBSTACLE % "█"
                    color = [0, 0, 0];
                case DSLState.UNKNOWN % "▓"
                    color = [255, 120, 120];
                case DSLState.EMPTY % "░"
                    color = [255, 255, 255];

                case DSLState.START % "ⓢ"
                    color = [120, 0, 120];
                case DSLState.GOAL % "♛"
                    color = [255, 0, 0];
                case DSLState.POSITION % "☺"
                    color = [0, 0, 255];

                case DSLState.VISITED % "╬"
                    color = [0, 255, 0];
                case DSLState.PATH % "≡"
                    color = [255, 0, 0];
            end
        end
    end
end


