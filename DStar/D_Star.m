classdef D_Star < handle
    properties
        map
        open_list = State.empty;
    end

    methods
        function obj = D_Star(map)
            obj.map = map;
        end

        function [state, index] = min_state(obj)
            if isempty(obj.open_list)
                state = State.empty;
                index = -1;
            else
                index = 1;
                state = obj.open_list(index);
                for i = 2:size(obj.open_list)
                    if obj.open_list(i).k < state.k
                        state = obj.open_list(i);
                        index = i;
                    end
                end
            end
        end

        function k = process_state(obj)
            [X, index] = obj.min_state;
            if isempty(X)
                error("Path not found")
            else
                k_old = obj.k_min;
                obj.remove(X, index);
                neighbors = obj.map.get_neighbors(X);
                if k_old < X.h
                    for y = neighbors
                        if y.h <= k_old && X.h > y.h + X.cost(y)
                            X.parent = y;
                            X.h = y.h + X.cost(y);
                        end
                    end
                elseif k_old == X.h
                    for y = neighbors
                        if y.tag == "new" || y.parent == X && y.h ~= X.h + X.cost(y)|| y.parent ~= X && y.h > X.h + X.cost(y)
                            y.parent = X;
                            obj.insert(y, X.h + X.cost(y));
                        end
                    end
                else
                    for y = neighbors
                        if y.tag == "new" || y.parent == X && y.h ~= X.h + X.cost(y)
                            y.parent = X;
                            obj.insert(y, X.h + X.cost(y));
                        elseif y.parent ~= X && y.h > X.h + X.cost(y)
                            obj.insert(y, X.h);
                        elseif y.parent ~= X && X.h > y.h + X.cost(y) && y.tag == "close" && y.h > k_old
                            obj.insert(y, y.h)
                        end
                    end
                end

                k = obj.k_min;
            end
        end

        function k = k_min(obj)
            if isempty(obj.open_list)
                k = -1;
            else
                k = min([obj.open_list.k]);
            end
        end

        function insert(obj, state, h_new)
            if state.tag == "new"
                state.k = h_new;
            elseif state.tag == "open"
                state.k = min(state.k, h_new);
            elseif state.tag == "close"
                state.k = min(state.h, h_new);
            end
            state.h = h_new;
            state.tag = "open";
            obj.open_list(end+1) = state;
        end

        function remove(obj, state, index)
            if state.tag == "open"
                state.tag = "close";
            end
            obj.open_list = [obj.open_list(1:index-1) obj.open_list(index+1:end)];
        end

        function modify(obj, state)
            obj.modify_cost(state);
            while true
                k_min = obj.process_state();
                if k_min >= state.h
                    break
                end
            end
        end

        function modify_cost(obj, state)
            if state.tag == "close"
                obj.insert(state, state.parent.h + state.cost(state.parent))
            end
        end

        function run(obj, start, goal)
            obj.open_list(end+1) = goal;
            while true
                obj.process_state();
                if start.tag == "close"
                    break
                end
            end
            start.state = "s";
            s = start;
            while s ~= goal
                s.state = "s";
                s = s.parent;
            end
            s.state = "e";
            obj.map.print_map();

            tmp = start;
            obj.map.set_obstacle([9 3; 9 4; 9 5; 9 6; 9 7; 9 8]);
            
            while tmp ~= goal
                tmp.state = "*";
                obj.map.print_map();
                if tmp.parent.state == "#"
                    obj.modify(tmp);
                    continue
                end
                tmp = tmp.parent;
            end
            tmp.state = "e";
        end
    end
end