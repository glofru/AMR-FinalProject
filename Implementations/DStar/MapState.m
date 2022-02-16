classdef MapState
    % enumeration map states
    enumeration
        OBSTACLE
        EMPTY
        
        START
        CURPOS
        GOAL
        
        PATH
    end
    
    methods
        % return the value of the state obj
        function chr = str(obj)
            switch obj
                case MapState.OBSTACLE
                    chr = "█";
                case MapState.EMPTY
                    chr = "░";
                case MapState.START
                    chr = "ⓢ";
                case MapState.CURPOS
                    chr = "a";
                case MapState.GOAL
                    chr = "♛";
                case MapState.PATH
                    chr = "≡";
                    
                otherwise
                    error("Wrong value!");
            end
        end

        % return the color associated to the state obj
        function color = getColor(obj)
            switch obj
                case MapState.OBSTACLE % "█"
                    color = [0, 0, 0];
                case MapState.EMPTY % "░"
                    color= [1, 1, 1];
                case MapState.START % "ⓢ"
                    color = [0.5, 0, 0.5];
                case MapState.CURPOS % "a"
                    color = [0.5, 0.5, 0];
                case MapState.GOAL % "♛"
                    color = [0.5, 0, 0];
                case MapState.PATH % "≡"
                    color = [1, 0, 0];
                otherwise
                    error("Wrong value!");
            end
        end
    end
end