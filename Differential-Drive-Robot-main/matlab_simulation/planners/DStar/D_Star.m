classdef D_Star < handle
    properties
        globalMap;

        localMap;
        currPos;
        goal;
        moves;

        map_limit;
        open_list;
        maxIter;
        obstacles;
    end

    methods
        function obj = D_Star(init_state, sampling_time, limit, goal,...
                map, resolution, maxIter)
            obj.globalMap = map;
            obj.moves = [[1; 0], [1; 1], [0; 1], [-1; 1], [-1; 0], [-1; -1], [0; -1], [1; -1]];

            obj.map_limit = limit;
            obj.goal = int16(goal/resolution);
            obj.maxIter = maxIter; % TODO
            start = [int16(init_state(1)/resolution) int16(init_state(2)/resolution)];
            
            size_x = size(map, 1);
            size_y = size(map, 2);
            
            for i=1:size_x
                for j=1:size_y
                    if (map(i, j) < 250)
                        obj.obstacles = [obj.obstacles, [i;j]];
                    end
                end
            end

            % initialize map
            obj.localMap = Map(size_x, size_y, obj.obstacles, Map.TYPE_UNKNOWN);
            
            obj.currPos = obj.localMap.map(start(1), start(2));
            obj.currPos.state = MapState.START;
            obj.goal = obj.localMap.map(obj.goal(1), obj.goal(2));
            obj.goal.state = MapState.GOAL;
            
            obj.open_list = OpenList();
            obj.open_list.insert(obj.goal);
        end
        
        function s = neighbors(obj, X)
            s = [State.empty];
            pos = [X.x; X.y];
            
            for m=obj.moves
                succ_pos = pos + m;
                x = succ_pos(1);
                y = succ_pos(2);

                if obj.localMap.isInside(x, y) && ~obj.localMap.isObstacle(x, y)
                    s(end+1) = obj.localMap.map(x, y);
                end
            end
        end

        function res = process_state(obj)
            [Kold, X] = obj.open_list.min_state();
            if isempty(X)
                error("Path not found")
            end
            obj.remove(X);
            

            if X.state ~= MapState.GOAL && X.state ~= MapState.START
                X.state = MapState.VISITED;
            end
            
            obj.localMap.plotMapDStar();
            
            succ = obj.neighbors(X);
            if Kold < X.h
                for Y=succ
                    if Y.h <= Kold && X.h > Y.h + X.cost(Y)
                        X.parent = Y;
                        X.h = Y.h + X.cost(Y);
                    end
                end
            elseif Kold == X.h
                for Y=succ
                    if Y.tag == StateTag.NEW || ...
                            (~isempty(Y.parent) && Y.parent == X && Y.h ~= X.h + X.cost(Y)) || ...
                            (~isempty(Y.parent) && Y.parent ~= X && Y.h > X.h + X.cost(Y))
                        Y.parent = X;
                        obj.insert(Y, X.h + X.cost(Y));
                    end
                end
            else
                for Y=succ
                    if Y.tag == StateTag.NEW ||...
                            (Y.parent == X && Y.h ~= X.h + X.cost(Y))
                        Y.parent = X;
                        obj.insert(Y, X.h + X.cost(Y));
                    else
                        if Y.parent ~= X && Y.h > X.h + X.cost(Y)
                            obj.insert(Y, X.h);
                        else
                            if Y.parent ~= X && X.h > Y.h + X.cost(Y) && ...
                                    Y.tag == StateTag.CLOSED && ...
                                    Y.h > Kold
                                obj.insert(Y, Y.h);
                            end
                        end
                    end
                end
            end

            res = obj.open_list.get_kmin();
        end

        function insert(obj, state, h_new)
            if state.tag == StateTag.NEW
                state.k = h_new;
            elseif state.tag == StateTag.OPEN
                state.k = min(state.k, h_new);
            elseif state.tag == StateTag.CLOSED
                state.k = min(state.h, h_new);
            end
            state.h = h_new;
            state.tag = StateTag.OPEN;
            obj.open_list.insert(state);
        end

        function remove(obj, state)
            if state.tag == StateTag.OPEN
                state.tag = StateTag.CLOSED;
            end
            obj.open_list.remove(state);
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
            if state.tag == StateTag.CLOSED
                obj.insert(state, state.parent.h + state.cost(state.parent))
            end
        end

        function final_path = run(obj)
            final_path = ones(obj.maxIter, 6);
            dimension_path = 1;
            final_path(dimension_path, 1:2) = [obj.currPos.x, obj.currPos.y]; 

            obj.localMap.plotMapDStar();
            while obj.currPos.tag ~= StateTag.CLOSED
                obj.process_state();
            end

            while ~obj.currPos.parent.eq(obj.goal)
                %move to minPos
                obj.currPos = obj.currPos.parent;
                obj.currPos.state = MapState.PATH;

                dimension_path = dimension_path + 1;
                final_path(dimension_path,1:2) = [obj.currPos.x, obj.currPos.y]; 

                obj.localMap.plotMapDStar();

                % scan graph
                % is_changed = updateMap();

                % update graph
                % if is_changed
                %    update_edges_cost();
                %    compute_shortest_path();
                % end
                
            end

            final_path = final_path(1:dimension_path, :);
            
            if dimension_path >= obj.maxIter
                disp("No possible path!");
            else
                disp("Goal reached!");
            end
        end
        
    end
end