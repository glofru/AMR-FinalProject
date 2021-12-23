classdef Map < handle
    properties (Constant)
        MAP_OBSTACLE = "█";
        MAP_UNKNOWN = "▓";
        MAP_EMPTY = "░";
        MAP_POSITION = "☺";
        
        MAP_START = "ⓢ";
        MAP_GOAL = "♛";
        
        TYPE_KNOWN = 0;
        TYPE_UNKNOWN = 1;
    end
    
    properties
        %
        row
        %
        col
        
        %
        map
        
        %
        % | x1, x2, x3, ...
        % | y1, y2, y3, ...
        obstacles
    end

    methods
        function obj = Map(row, col, obstacles, type)
            obj.row = row;
            obj.col = col;
            
            switch type
                case Map.TYPE_KNOWN
                    obj.init_map_known();
                case Map.TYPE_UNKNOWN
                    obj.init_map_unknown();
                otherwise
                    error("Wrong!") % TODO
            end
            
            obj.obstacles = obstacles;
            obj.setObstacles(obstacles);
        end

        function init_map_known(obj)
            obj.map = State.empty(1, 0);
            for i=1:obj.row
                tmp = State.empty(0, 1);
                for j=1:obj.col
                    tmp(j) = State(i, j, Map.MAP_EMPTY);
                end
                obj.map = [obj.map; tmp];
            end
        end
        
        function init_map_unknown(obj)
            obj.map = State.empty(1, 0);
            for i=1:obj.row
                tmp = State.empty(0, 1);
                for j=1:obj.col
                    tmp(j) = State(i, j, Map.MAP_UNKNOWN);
                end
                obj.map = [obj.map; tmp];
            end
        end
        
        function setObstacles(obj, point_list)
            for point=point_list
                if obj.isInside(point(1), point(2))
                    obj.map(point(1), point(2)).state = Map.MAP_OBSTACLE;
                end
            end
        end
        
        function res = isInside(obj, x, y)
            if x < 1 || x > obj.row
                res = false;
                return;
            end
            if y < 1 || y > obj.col
                res = false;
                return;
            end
            res = true;
        end
        
        function res = isObstacle(obj, x, y)
            res = false;
            for o=obj.obstacles
                if all(o==[x; y])
                    res = true;
                    break
                end
            end
        end
        
        % ### PLOT FUNCTIONS ### %
        
        function plotMap(obj)
            outHeader = "";
            for i=1:(obj.col+2)
                outHeader = outHeader + Map.MAP_OBSTACLE;
            end
            disp(outHeader);

            for i=1:obj.row
                out = "";
                for j=1:obj.col
                    out = out + obj.map(i, j).state;
                end
                disp(Map.MAP_OBSTACLE+out+Map.MAP_OBSTACLE);
            end

            disp(outHeader+newline);
        end
    end
end