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

            % inizialize map
            obj.localMap = Map(size_x, size_y, obj.obstacles, Map.TYPE_UNKNOWN);
            
            obj.currPos = obj.localMap.map(start(1), start(2));
            obj.currPos.state = MapState.START;
            obj.goal = obj.localMap.map(obj.goal(1), obj.goal(2));
            obj.goal.state = MapState.GOAL;
            obj.goal.h = 0;
            
            obj.open_list = OpenList();
            obj.open_list = obj.open_list.insert(obj.goal);
        end
        
        function Ls = successor(obj, X)
            Ls = OpenList();
            pos = [X.x; X.y];
            
            for m=obj.moves
                
                succ_pos = pos + m;
                x = succ_pos(1);
                y = succ_pos(2);

                if ~obj.localMap.isInside(x, y) || obj.localMap.isObstacle(x, y)
                    continue
                end

                if ~Ls.has(obj.localMap.map(x, y))
                    Ls = Ls.insert(obj.localMap.map(x, y));
                end
            end
        end

        function res = process_state(obj)
            [obj.open_list, X, Kold] = obj.open_list.pop();
            X.tag = StateTag.CLOSED;
            if X.state ~= MapState.GOAL
                X.state = MapState.POSITION;
            end
            obj.localMap.plotMap();

            if isempty(X)
                error("Path not found")
            end
            
            succ = obj.successor(X);
            if Kold < X.h
                for Y=succ
                    if Y.h < Kold && X.h > Y.h + X.cost(Y)
                        X.parent = Y;
                        X.h = Y.h + X.cost(Y);
                    end
                end
            end
            
            if Kold == X.h
                for i=1:size(succ.queueS, 2)
                    Y = succ.queueS(i);
                    if Y.tag == StateTag.NEW || ...
                            (~isempty(Y.parent) && Y.parent == X && Y.h ~= X.h + X.cost(Y)) || ...
                            (~isempty(Y.parent) && ~(Y.parent == X) && Y.h > X.h + X.cost(Y))
                        Y.parent = X;
                        Y.h = X.h + X.cost(Y);
                        Y.tag = StateTag.OPEN;
                        obj.open_list = obj.open_list.insert(Y);
                    end
                end
            else
                for i=1:size(succ.queueS, 2)
                    Y = succ.queueS(i);
                    if Y.tag == StateTag.NEW ||...
                            (Y.parent == X && Y.h ~= X.h + X.cost(Y))
                        Y.parent = X;
                        Y.h = X.h + X.cost(Y);
                        Y.tag = StateTag.OPEN;
                        obj.open_list = obj.open_list.insert(Y);
                    else
                        if ~(Y.parent == X) && Y.h > X.h + X.cost(Y)
                            X.tag = StateTag.OPEN;
                            obj.open_list = obj.open_list.insert(X);
                        else
                            if ~(Y.parent == X) && X.h > Y.h + Y.cost(X) && ...
                                    Y.tag == StateTag.CLOSED && ...
                                    Y.h > Kold
                                Y.tag = StateTag.OPEN;
                                obj.open_list = obj.open_list.insert(Y);
                            end
                        end
                    end
                end
            end

            if X.state ~= MapState.GOAL
                X.state = MapState.VISITED;
            end
            obj.localMap.plotMap();
            [~, res] = obj.open_list.top();
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

            PSret = 0;
            obj.localMap.plotMap();
            while obj.currPos.tag ~= StateTag.CLOSED && PSret ~= -1
                PSret = obj.process_state();
            end
            
            if obj.currPos.tag ~= StateTag.CLOSED && PSret == -1
                error("No possible path")
            end

            while ~obj.currPos.parent.eq(obj.goal)
                %move to minPos
                obj.currPos = obj.currPos.parent;
                obj.currPos.state = MapState.PATH;

                dimension_path = dimension_path + 1;
                final_path(dimension_path,1:2) = [obj.currPos.x, obj.currPos.y]; 

                obj.localMap.plotMap();

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