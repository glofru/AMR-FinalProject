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
        function obj = D_Star(init_state, ~, limit, goal,...
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
            obj.localMap = DMap(size_x, size_y, obj.obstacles);
            
            obj.currPos = obj.localMap.map(start(1), start(2));
            obj.currPos.state = DMapState.START;
            obj.goal = obj.localMap.map(obj.goal(1), obj.goal(2));
            obj.goal.state = DMapState.GOAL;
            
            obj.open_list = OpenList(obj.goal);
        end

        function res = process_state(obj)
            [Kold, X] = obj.open_list.min_state();
            if isempty(X)
                error("Path not found")
            end
            obj.open_list.remove(X);
            
            obj.localMap.plot(X);
            
            succ = obj.localMap.neighbors(X, obj.moves);
            if Kold < X.h
                for Y=succ
                    if Y.h <= Kold && X.h > Y.h + X.cost(Y)
                        X.parent = Y;
                        X.h = Y.h + X.cost(Y);
                    end
                end
            elseif Kold == X.h
                for Y=succ
                    if Y.tag == DStateTag.NEW || ...
                            (~isempty(Y.parent) && Y.parent == X && Y.h ~= X.h + X.cost(Y)) || ...
                            (~isempty(Y.parent) && Y.parent ~= X && Y.h > X.h + X.cost(Y))
                        Y.parent = X;
                        obj.open_list.insert(Y, X.h + X.cost(Y));
                    end
                end
            else
                for Y=succ
                    if Y.tag == DStateTag.NEW ||...
                            (Y.parent == X && Y.h ~= X.h + X.cost(Y))
                        Y.parent = X;
                        obj.open_list.insert(Y, X.h + X.cost(Y));
                    else
                        if Y.parent ~= X && Y.h > X.h + X.cost(Y)
                            obj.open_list.insert(Y, X.h);
                        else
                            if Y.parent ~= X && X.h > Y.h + X.cost(Y) && ...
                                    Y.tag == DStateTag.CLOSED && ...
                                    Y.h > Kold
                                obj.open_list.insert(Y, Y.h);
                            end
                        end
                    end
                end
            end

            res = obj.open_list.get_kmin();
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
            if state.tag == DStateTag.CLOSED
                obj.open_list.insert(state, state.parent.h + state.cost(state.parent))
            end
        end

        function final_path = run(obj)
            final_path = ones(obj.maxIter, 6);
            dimension_path = 1;
            final_path(dimension_path, 1:2) = [obj.currPos.x, obj.currPos.y]; 

            obj.localMap.plot();
            while obj.currPos.tag ~= DStateTag.CLOSED
                obj.process_state();
            end

            while ~obj.currPos.parent.eq(obj.goal)
                %move to minPos
                obj.currPos = obj.currPos.parent;
                obj.currPos.state = DMapState.PATH;

                dimension_path = dimension_path + 1;
                final_path(dimension_path,1:2) = [obj.currPos.x, obj.currPos.y]; 

                obj.localMap.plot();

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