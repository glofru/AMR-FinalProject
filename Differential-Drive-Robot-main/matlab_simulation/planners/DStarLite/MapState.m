classdef MapState
    % enumeration map states
    
    enumeration
        OBSTACLE
        UNKNOWN
        EMPTY
        
        START
        GOAL
        POSITION
        
        VISITED
        PATH
    end
    
    methods
        % return the value of the state obj
        function chr = str(obj)
            switch obj
                case MapState.OBSTACLE
                    chr = "█";
                case MapState.UNKNOWN
                    chr = "▓";
                case MapState.EMPTY
                    chr = "░";
                    
                case MapState.START
                    chr = "ⓢ";
                case MapState.GOAL
                    chr = "♛";
                case MapState.POSITION
                    chr = "☺";
                    
                case MapState.VISITED
                    chr = "╬";
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
                case MapState.UNKNOWN % "▓"
                    color = [255, 120, 120];
                case MapState.EMPTY % "░"
                    color= [255, 255, 255];
                    
                case MapState.START % "ⓢ"
                    color = [120, 0, 120];
                case MapState.GOAL % "♛"
                    color = [255, 0, 0];
                case MapState.POSITION % "☺"
                    color = [0, 0, 255];
                    
                case MapState.VISITED % "╬"
                    color = [0, 255, 0];
                case MapState.PATH % "≡"
                    color = [255, 0, 0];
                    
                otherwise
                    error("Wrong value!");
            end
        end
    end
end