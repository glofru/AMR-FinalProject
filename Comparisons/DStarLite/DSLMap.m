classdef DSLMap < handle
    %
    %
    
    properties(Constant) % enumeration
        TYPE_KNOWN = 1;
        TYPE_UNKNOWN = 2;
    end
    
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
        function init_map(obj, chr)
            obj.map = DSLState();
            obj.map(obj.row, obj.col) = DSLState();
%             obj.map = DSLState.empty(1, 0);
%             for i=1:obj.row
%                 tmp = DSLState.empty(0, 1);
%                 for j=1:obj.col
%                     tmp(j) = DSLState(i, j, chr, obj.cost);
%                 end
%                 obj.map = [obj.map; tmp];
%             end
        end
    end

    methods
        % DSLMap constructor
        function obj = DSLMap(row, col, obstacles, type, cost)
            obj.row = row;
            obj.col = col;
            obj.cost = cost;
            
            switch type
                case DSLMap.TYPE_KNOWN
                    obj.init_map(DSLState.EMPTY);
                case DSLMap.TYPE_UNKNOWN
                    obj.init_map(DSLState.UNKNOWN);
                otherwise
                    error("Wrong Map Type!")
            end
            
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


