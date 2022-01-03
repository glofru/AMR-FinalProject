classdef Map < handle
    properties (Constant)
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
                    error("Wrong map type")
            end
            
            obj.obstacles = obstacles;
            obj.setObstacles(obstacles);
        end

        function init_map_known(obj)
            obj.map = State.empty(1, 0);
            for i=1:obj.row
                tmp = State.empty(0, 1);
                for j=1:obj.col
                    tmp(j) = State(i, j, MapState.EMPTY);
                end
                obj.map = [obj.map; tmp];
            end
        end
        
        function init_map_unknown(obj)
            obj.map = State.empty(1, 0);
            for i=1:obj.row
                tmp = State.empty(0, 1);
                for j=1:obj.col
                    tmp(j) = State(i, j, MapState.UNKNOWN);
                end
                obj.map = [obj.map; tmp];
            end
        end
        
        function setObstacles(obj, point_list)
            for point=point_list
                if obj.isInside(point(1), point(2))
                    obj.map(point(1), point(2)).state = MapState.OBSTACLE;
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
            for o = obj.obstacles
                if all(o==[x; y])
                    res = true;
                    break
                end
            end
        end
        
        % ### PLOT FUNCTIONS ### %
        
        function rgbImage = buildImageMap(obj)
            rgbImage = zeros(obj.row, obj.col, 3) + 255;

            for i = 1:obj.row
                for j = 1:obj.col
                    rgbImage(i,j,:) = obj.map(i, j).state.getColor();
                end
            end
        end

        function rgbImage = buildImageMapDStar(obj)
            rgbImage = zeros(obj.row, obj.col, 3) + 255;

            for i = 1:obj.row
                for j = 1:obj.col
                    state = obj.map(i, j).state;
                    tag = obj.map(i, j).tag;

                    if state == MapState.VISITED || (state == MapState.UNKNOWN && tag == StateTag.OPEN)
                        rgbImage(i,j,:) = tag.getColor();
                    else
                        rgbImage(i,j,:) = state.getColor();
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

        function plotMapDStar(obj)
            J = obj.buildImageMapDStar();
            %J = imrotate(rgbImage,90);
            %J = imresize( J , 100);
            imshow(J,'InitialMagnification',1000);
        end
        
        function plotMap_g(obj)
            outHeader = "";
            for i=1:(obj.col+2)
                outHeader = outHeader + disp(MapState.OBSTACLE);
            end
            disp(outHeader);

            for i=1:obj.row
                out = "";
                for j=1:obj.col
                    out = out + obj.map(i, j).g + " ";
                end
                disp(disp(MapState.OBSTACLE) + out + disp(MapState.OBSTACLE));
            end

            disp(outHeader + newline);
        end
        
        function plotMap_rhs(obj)
            outHeader = "";
            for i=1:(obj.col+2)
                outHeader = outHeader + disp(MapState.OBSTACLE);
            end
            disp(outHeader);

            for i=1:obj.row
                out = "";
                for j=1:obj.col
                    out = out + obj.map(i, j).rhs+" ";
                end
                disp(disp(Map.MAP_OBSTACLE) + out + disp(MapState.OBSTACLE));
            end

            disp(outHeader + newline);
        end
        
        function plotMap_k(obj)
            outHeader = "";
            for i=1:(obj.col + 2)
                outHeader = outHeader + disp(MapState.OBSTACLE);
            end
            disp(outHeader);

            for i=1:obj.row
                out = "";
                for j=1:obj.col
                    out = out + obj.map(i, j).k+" ";
                end
                disp(disp(MapState.OBSTACLE) + out + disp(MapState.OBSTACLE));
            end

            disp(outHeader + newline);
        end
    end
end