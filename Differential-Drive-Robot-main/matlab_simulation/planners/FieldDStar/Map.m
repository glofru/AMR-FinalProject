classdef Map < handle
    % Class to keep and work with the map
    %
    
    properties (Constant)
        TYPE_KNOWN = 0;
        TYPE_UNKNOWN = 1;
    end
    
    properties
        % num of map's rows
        row %double {mustBePositive, mustBeInteger}
        % num of map's cols
        col %double {mustBePositive, mustBeInteger}
        
        % row x col matrix of State
        map %(:, :) {}
        
        % matrix 2 x N list of obstacles
        % | x1, x2, x3, ...
        % | y1, y2, y3, ...
        obstacles %(2, :) {mustBePositive, mustBeInteger}
        
        %
        cost
    end

    methods
        % map constructor
        function obj = Map(row, col, obstacles, type, cost)
            arguments
                % num of map's rows
                row %double {mustBePositive, mustBeInteger}
                % num of map's cols
                col %double {mustBePositive, mustBeInteger}
                
                % matrix 2 x N list of obstacles
                % | x1, x2, x3, ...
                % | y1, y2, y3, ...
                obstacles %(2, :) {mustBePositive, mustBeInteger} = []
                
                % type of the non obstacles tiles
                type %double {} = Map.TYPE_UNKNOWN
                
                cost = 1;
            end
            
            obj.row = row;
            obj.col = col;
            obj.cost = cost;
            
            switch type
                case Map.TYPE_KNOWN
                    obj.init_map(MapState.EMPTY);
                case Map.TYPE_UNKNOWN
                    obj.init_map(MapState.UNKNOWN);
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

        % set a list of obstacles inside the map
        function setObstacles(obj, point_list)
            arguments
                obj
                
                % matrix 2 x N list of obstacles
                % | x1, x2, x3, ...
                % | y1, y2, y3, ...
                point_list %(2, :) {mustBePositive, mustBeInteger}
            end
            
            for point=point_list
                if obj.isInside(point(1), point(2))
                    obj.map(point(1), point(2)).state = MapState.OBSTACLE;
                end
            end
        end
        
        % check if (x, y) is inside the map
        function res = isInside(obj, x, y)
            arguments
                obj
                
                % x and y position of the position to check
                x %double {mustBeNumeric}
                y %double {mustBeNumeric}
            end
            
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
        
        % check if (x, y) is an obstacle
        function res = isObstacle(obj, x, y)
            arguments
                obj
                
                % x and y position of the position to check
                x %double {mustBeNumeric}
                y %double {mustBeNumeric}
            end
            
            res = false;
            for o = obj.obstacles
                if all(o==[x; y])
                    res = true;
                    break
                end
            end
        end
        
        % ### PLOT FUNCTIONS ### %
        
        % generate the map image
        function rgbImage = buildImageMap(obj)
            rgbImage = zeros(obj.row, obj.col, 3)+255;

            for i = 1:obj.row
                for j = 1:obj.col
                    rgbImage(i,j,:) = obj.map(i, j).state.getColor();
                end
            end
        end
        
        % plot the map image
        function plotMap(obj)
            J = obj.buildImageMap();
            %J = imrotate(rgbImage,90); % TODO decide if to keep
            %J = imresize( J , 100); % TODO decide if to keep
            imshow(J,'InitialMagnification',1000);
        end
    end
end