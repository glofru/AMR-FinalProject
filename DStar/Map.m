classdef Map < handle
    properties (Constant)
        MAP_OBSTACLE = "#";
        MAP_EMPTY = ".";
        MAP_PATH = "P";
        
        MAP_START = "S";
        MAP_GOAL = "G";
    end
    
    properties
        row
        col
        map
        obsts
    end

    methods
        function obj = Map(row, col, obsts)
            obj.row = row;
            obj.col = col;
            obj.init_map();
            obj.obsts = obsts;
            obj.set_obstacle(obsts);
        end

        function init_map(obj)
            obj.map = State.empty(1, 0);
            for i=1:obj.row
                tmp = State.empty(0, 1);
                for j=1:obj.col
                    tmp(j) = State(i, j);
                end
                obj.map = [obj.map; tmp];
            end
        end

        function print_map(obj)
            tmp = "";
            for i=1:obj.row
                for j=1:obj.col
                    tmp = tmp + obj.map(i, j).state + " ";
                end
                tmp = tmp + "\n";
            end
            tmp = tmp + "\n";
            fprintf(tmp);
        end
        
        function print_map_tag(obj)
            tmp = "";
            for i=1:obj.row
                for j=1:obj.col
                    switch(obj.map(i, j).tag)
                        case State.TAG_NEW
                            tmp = tmp + "N ";
                            continue
                        case State.TAG_OPEN
                            tmp = tmp + "O ";
                            continue
                        case State.TAG_CLOSED
                            tmp = tmp + "C ";
                            continue
                    end
                end
                tmp = tmp + "\n";
            end
            tmp = tmp + "\n";
            fprintf(tmp);
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
            for o=obj.obsts
                if all(o==[x; y])
                    res = true;
                    break
                end
            end
        end

        function neighbors = get_neighbors(obj, state)
            neighbors = State.empty;
            for i=-1:1
                for j=-1:1
                    if i == 0 && j == 0
                        continue
                    elseif state.x + i < 1 || state.x + i > obj.row || state.y + j < 1 || state.y + j > obj.col
                        continue
                    else
                        neighbors(end+1) = obj.map(state.x + i, state.y + j);
                    end
                end
            end
        end

        function set_obstacle(obj, point_list)
            for point=point_list
                if obj.isInside(point(1), point(2))
                    obj.map(point(1), point(2)).state = Map.MAP_OBSTACLE;
                end
            end
        end
    end

end