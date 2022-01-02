classdef MapState
    enumeration
        OBSTACLE
        UNKNOWN
        EMPTY
        POSITION
        
        START
        GOAL
        
        VISITED
        PATH
    end
    methods
        function d = disp(obj)
            switch obj
                case MapState.OBSTACLE
                    d = "█";
                case MapState.GOAL
                    d = "♛";
                case MapState.PATH
                    d = "≡";
                case MapState.VISITED
                    d = "╬";
                case MapState.EMPTY
                    d = "░";
                case MapState.UNKNOWN
                    d = "▓";
                case MapState.POSITION
                    d = "☺";
                case MapState.START
                    d = "ⓢ";
            end
        end

        function color = getColor(obj)
            switch obj
                case MapState.OBSTACLE % "█"
                    color = [0, 0, 0];
                    
                case MapState.GOAL % "♛"
                    color = [255, 0, 0];
                    
                case MapState.PATH % "≡"
                    color = [255, 0, 0];
                    
                case MapState.VISITED % "╬"
                    color = [0, 255, 0];
                    
                case MapState.EMPTY % "░"
                    color= [255, 255, 255];
                    
                case MapState.UNKNOWN % "▓"
                    color = [255, 120, 120];
                    
                case MapState.POSITION % "☺"
                    color = [0, 0, 255];
                    
                case MapState.START % "ⓢ"
                    color = [120, 0, 120];
            end
        end
    end
end