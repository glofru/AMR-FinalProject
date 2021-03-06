classdef FDState < handle
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
        % FDState constructor
        function obj = FDState(x, y, state, cost)
            obj.x = x;
            obj.y = y;
            obj.state = state;
            obj.cost = cost;
            
            obj.g = Inf;
            obj.rhs = Inf;
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
                s1 = sa;
                s2 = sb;
            else
                s1 = sb;
                s2 = sa;
            end
            
            c = obj.c(sa);
            b = obj.c(sb);
            
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

        function setPos(obj, x, y, cost)
            obj.x = x;
            obj.y = y;
            obj.cost = cost;
        end
        
        
        % check if 2 states are equal
        function e = eq(obj, s)
            e = (obj.x == s.x && obj.y == s.y);
        end
        
        
        % return the color of the state
        function color = getColor(obj)
            switch obj.state
                case FDState.OBSTACLE % "█"
                    color = [0, 0, 0];
                case FDState.UNKNOWN % "▓"
                    color = [255, 120, 120];
                case FDState.EMPTY % "░"
                    color = [255, 255, 255];

                case FDState.START % "ⓢ"
                    color = [120, 0, 120];
                case FDState.GOAL % "♛"
                    color = [255, 0, 0];
                case FDState.POSITION % "☺"
                    color = [0, 0, 255];

                case FDState.VISITED % "╬"
                    color = [0, 255, 0];
                case FDState.PATH % "≡"
                    color = [255, 0, 0];
            end
        end
    end
end


