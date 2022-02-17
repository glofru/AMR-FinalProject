classdef State < handle
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
        FUTUREPATH = "x";
        
        OPEN = "o";
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
                case State.OBSTACLE % "█"
                    color = [0, 0, 0];
                case State.UNKNOWN % "▓"
                    color = [0.75, 0.75, 0.75];
                case State.EMPTY % "░"
                    color = [1, 1, 1];

                case State.START % "ⓢ"
                    color = [0.5, 0, 0.5];
                case State.GOAL % "♛"
                    color = [1, 0, 0];
                case State.POSITION % "☺"
                    color = [0, 0, 1];

                case State.VISITED % "╬"
                    color = [0, 1, 0];
                case State.PATH % "≡"
                    color = [1, 0, 0];
                case State.FUTUREPATH % "x"
                    color = [0.95, 0.95, 0];
                    
                case State.OPEN % "o"
                    color = [0.5, 0.5, 0];
            end
        end
    end

    methods
        % State constructor
        function obj = State(x, y, state, cost)
            arguments
                % x coord
                x
                % y coord
                y
                
                % state of this cell
                state {} = State.UNKNOWN
                % cost of a step
                cost = 1
            end
            obj.x = x;
            obj.y = y;
            obj.state = state;
            obj.cost = cost;
            
            obj.g = Inf;
            obj.rhs = Inf;
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
            e =  (obj.x == s.x && obj.y == s.y);
        end
        
        
        % return the color of the state
        function color = getColor(obj)
            color = State.returnStateColor(obj.state);
        end
    end
end


