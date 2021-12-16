classdef Map < handle
    properties
        row
        col
        map
    end

    methods
        function obj = Map(row, col)
            obj.row = row;
            obj.col = col;
            obj.init_map();
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

        function neighbors = get_neighbors(obj, state)
            neighbors = State.empty;
            for i=-1:1
                for j=-1:1
                    if i == 0 && j == 0
                        continue
                    elseif state.x + i < 1 || state.x + i > obj.row
                        continue
                    elseif state.y + j < 1 || state.y + j > obj.col
                        continue
                    else
                        neighbors(end+1) = obj.map(state.x + i, state.y + j);
                    end
                end
            end
        end

        function set_obstacle(obj, point_list)
            [r, ~] = size(point_list);
            for i=1:r
                point = point_list(i, :);
                if point(1) < 1 || point(1) > obj.row || point(2) < 1 || point(2) > obj.col
                    continue
                else
                    obj.map(point(1), point(2)).state = "#";
                end
            end
        end
    end

end