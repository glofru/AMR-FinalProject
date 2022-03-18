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
        OPEN = "o";
        PATH = "≡";
        FUTUREPATH = "x";
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
    
    methods (Static)
        % return the color of the state
        function color = returnStateColor(state)
            switch state
                case DSLState.OBSTACLE % "█"
                    color = [0, 0, 0];
                case DSLState.UNKNOWN % "▓"
                    color = [0.75, 0.75, 0.75];
                case DSLState.EMPTY % "░"
                    color = [1, 1, 1];

                case DSLState.START % "ⓢ" init pos
                    color = [0, 0, 1];
                case DSLState.GOAL % "♛" goal pos
                    color = [1, 0, 0];
                case DSLState.POSITION % "☺" curr pos
                    color = [1, 1, 0];

                case DSLState.VISITED % "╬" inserd in the priority queue
                    color = [0.2, 0.8, 0.2];
                case DSLState.OPEN % "o" cell opened in computeShortestPath
                    color = [0.5, 0, 0.5];
                case DSLState.PATH % "≡" passed cell
                    color = [1, 0, 0];
                case DSLState.FUTUREPATH % "x" future path cell
                    color = [0.2, 0.6, 1];
            end
        end
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
            color = DSLState.returnStateColor(obj.state);
        end
    end
end


