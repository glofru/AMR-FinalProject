classdef DSLMap < handle
    %
    %
    
    properties
        % num of map's rows
        row
        % num of map's cols
        col
        
        % row x col matrix of States
        map
        
        % matrix 2 x N list of obstacles
        % | x1, x2, x3, ...
        % | y1, y2, y3, ...
        obstacles
        
        % cost of a step
        cost
    end
    
    methods(Access=private)
        function init_map(obj)
            obj.map = DSLState();
            obj.map(obj.row, obj.col) = DSLState();
        end
    end

    methods
        % DSLMap constructor
        function obj = DSLMap(row, col, obstacles, cost)
            obj.row = row;
            obj.col = col;
            obj.cost = cost;
            
            obj.init_map();
            
            obj.obstacles = obstacles;
            for point = obj.obstacles
                if obj.isInside(point(1), point(2))
                    obj.map(point(1), point(2)).state = DSLState.OBSTACLE;
                end
            end
        end
        
        
        % check if (x, y) is inside the map
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
        
        % check if (x, y) is an obstacle
        function res = isObstacle(obj, x, y)
            res = (obj.map(x, y).state == DSLState.OBSTACLE);
        end
        
        
        % generate the map image
        function rgbImage = buildImageMap(obj)
            rgbImage = zeros(obj.row, obj.col, 3)+255;

            for i = 1:obj.row
                for j = 1:obj.col
                    rgbImage(i,j,:) = obj.map(i, j).getColor();
                end
            end
        end
        
        % plot the map image
        function plot(obj)
            J = obj.buildImageMap();
            imshow(J,'InitialMagnification',1000);
        end
    end
end


