classdef DLMapState
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
                case DLMapState.OBSTACLE
                    chr = "█";
                case DLMapState.UNKNOWN
                    chr = "▓";
                case DLMapState.EMPTY
                    chr = "░";
                    
                case DLMapState.START
                    chr = "ⓢ";
                case DLMapState.GOAL
                    chr = "♛";
                case DLMapState.POSITION
                    chr = "☺";
                    
                case DLMapState.VISITED
                    chr = "╬";
                case DLMapState.PATH
                    chr = "≡";
                    
                otherwise
                    error("Wrong value!");
            end
        end

        % return the color associated to the state obj
        function color = getColor(obj)
            switch obj
                case DLMapState.OBSTACLE % "█"
                    color = [0, 0, 0];
                case DLMapState.UNKNOWN % "▓"
                    color = [255, 120, 120];
                case DLMapState.EMPTY % "░"
                    color= [255, 255, 255];
                    
                case DLMapState.START % "ⓢ"
                    color = [120, 0, 120];
                case DLMapState.GOAL % "♛"
                    color = [255, 0, 0];
                case DLMapState.POSITION % "☺"
                    color = [0, 0, 255];
                    
                case DLMapState.VISITED % "╬"
                    color = [0, 255, 0];
                case DLMapState.PATH % "≡"
                    color = [255, 0, 0];
                    
                otherwise
                    error("Wrong value!");
            end
        end
    end
end