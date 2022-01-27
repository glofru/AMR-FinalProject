classdef Map < handle
    properties (Constant)
        MAP_OBSTACLE = "█";
        MAP_UNKNOWN = "▓";
        MAP_EMPTY = "░";
        MAP_POSITION = "☺";
        
        MAP_START = "ⓢ";
        MAP_GOAL = "♛";
        
        MAP_VISITED = "╬"
        MAP_PATH = "≡"
        
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
        
        %
        cost
    end

    methods
        function obj = Map(row, col, obstacles, type, cost)
            arguments
                row
                col
                obstacles
                type
                
                cost = 1;
            end
            obj.row = row;
            obj.col = col;
            obj.cost = cost;
            
            switch type
                case Map.TYPE_KNOWN
                    obj.init_map(Map.MAP_EMPTY);
                case Map.TYPE_UNKNOWN
                    obj.init_map(Map.MAP_UNKNOWN);
                otherwise
                    error("Wrong!") % TODO
            end
            
            obj.obstacles = obstacles;
            obj.setObstacles(obstacles);
        end

        function init_map(obj, chr)
            obj.map = State.empty(1, 0);
            for i=1:obj.row
                tmp = State.empty(0, 1);
                for j=1:obj.col
                    tmp(j) = State(i, j, chr, obj.cost);
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
        
        function plotMap_old(obj) % TODO
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
        
        function rgbImage = buildImageMap(obj)
            rgbImage = zeros(obj.row, obj.col, 3)+255;

            for i = 1:obj.row
                for j = 1:obj.col
                    switch obj.map(i, j).state
                        case Map.MAP_OBSTACLE % "█"
                            rgbImage(i,j,:) = [0, 0, 0];
                            
                        case Map.MAP_GOAL % "♛"
                            rgbImage(i,j,:) = [255, 0, 0];
                            
                        case Map.MAP_PATH % "≡"
                            rgbImage(i,j,:) = [255, 0, 0];
                            
                        case Map.MAP_VISITED % "╬"
                            rgbImage(i,j,:) = [0, 255, 0];
                            
                        case Map.MAP_EMPTY % "░"
                            rgbImage(i,j,:) = [255, 255, 255];
                            
                        case Map.MAP_UNKNOWN % "▓"
                            rgbImage(i,j,:) = [255, 120, 120];
                            
                        case Map.MAP_POSITION % "☺"
                            rgbImage(i,j,:) = [0, 0, 255];
                            
                        case Map.MAP_START % "ⓢ"
                            rgbImage(i,j,:) = [120, 0, 120];
                    end
                end
            end
        end
        
        function plotMap(obj)
            J = obj.buildImageMap();
            %J = imrotate(rgbImage,90);
            %J = imresize( J , 100);
            imshow(J,'InitialMagnification',1000);
        end
        
        function plotMap_g(obj)
            outHeader = "";
            for i=1:(obj.col+2)
                outHeader = outHeader + Map.MAP_OBSTACLE;
            end
            disp(outHeader);

            for i=1:obj.row
                out = "";
                for j=1:obj.col
                    out = out + obj.map(i, j).g+" ";
                end
                disp(Map.MAP_OBSTACLE+out+Map.MAP_OBSTACLE);
            end

            disp(outHeader+newline);
        end
        
        function plotMap_rhs(obj)
            outHeader = "";
            for i=1:(obj.col+2)
                outHeader = outHeader + Map.MAP_OBSTACLE;
            end
            disp(outHeader);

            for i=1:obj.row
                out = "";
                for j=1:obj.col
                    out = out + obj.map(i, j).rhs+" ";
                end
                disp(Map.MAP_OBSTACLE+out+Map.MAP_OBSTACLE);
            end

            disp(outHeader+newline);
        end
        
        function plotMap_k(obj)
            outHeader = "";
            for i=1:(obj.col+2)
                outHeader = outHeader + Map.MAP_OBSTACLE;
            end
            disp(outHeader);

            for i=1:obj.row
                out = "";
                for j=1:obj.col
                    out = out + obj.map(i, j).k+" ";
                end
                disp(Map.MAP_OBSTACLE+out+Map.MAP_OBSTACLE);
            end

            disp(outHeader+newline);
        end
    end
end