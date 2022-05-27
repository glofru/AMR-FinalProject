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
        
        
        bptr
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

                case State.START % "ⓢ" init pos
                    color = [0, 0, 1];
                case State.GOAL % "♛" goal pos
                    color = [1, 0, 0];
                case State.POSITION % "☺" curr pos
                    color = [0.99, 0.99, 0];

                case State.VISITED % "╬" inserd in the priority queue
                    color = [0.2, 0.8, 0.2];
                case State.OPEN % "o" cell opened in computeShortestPath
                    color = [0.5, 0, 0.5];
                case State.PATH % "≡" passed cell
                    color = [1, 0, 0];
                case State.FUTUREPATH % "x" future path cell
                    color = [0.2, 0.6, 1];
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
            
            
            obj.bptr = State.empty(1, 0);
        end
        
        
        % return the key pair
        function K = calcKey(obj, Sstart)
            k1 = min(obj.g, obj.rhs + obj.h(Sstart));
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
        
        % s, sa, sb are neighbourds
        % c is the traversal cost of the center cell
        % b is the traversal cost of the bottom cell
        function vs = computeCost(obj, sa, sb)
            if (obj.x ~= sa.x && obj.y ~= sa.y)
                s1 = sb;
                s2 = sa;
            else
                s1 = sa;
                s2 = sb;
            end
            
            c = obj.c(s1);
            b = obj.c(s2);
            
            if (min(c,b) == inf)
                vs = inf;
            elseif (s1.g <= s2.g)
                vs = min(c, b) + s1.g;
            else
                f = s1.g - s2.g;
                
                if (f <= b)
                    if (c <= f)
                        vs = c*sqrt(2) + s2.g;
                    else
                        Y = min(f/(sqrt(c^2-f^2)), 1);
                        vs = c*sqrt(1+Y^2)+f*(1-Y)+s2.g;
                    end
                else
                    if (c <= b)
                        vs = c*sqrt(2)+s2.g;
                    else
                        X = 1-min(b/(sqrt(c^2-b^2)), 1);
                        vs = c*sqrt(1+(1-X)^2)+b*X+s2.g;
                    end
                end
            end
        end
        
        
        % check if 2 states are equal
        function e = eq(obj, s)
            e = ~isempty(obj) && ~isempty(s) && ...
                (obj.x == s.x && obj.y == s.y);
        end
        
        
        % return the color of the state
        function color = getColor(obj)
            color = State.returnStateColor(obj.state);
        end
    end
end


