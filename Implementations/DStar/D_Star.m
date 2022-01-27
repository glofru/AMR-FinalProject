classdef D_Star < handle
    properties
        map;
        open_list;
        moves;
    end

    methods
        function obj = D_Star(moves, map, goal)
            obj.moves = moves;
            
            obj.map = map;
            
            for stateC = map.map
                for state = stateC'
                    state.tag = State.TAG_NEW;
                end
            end
            
            obj.open_list = OpenList();
            goal.h = 0;
            obj.open_list = obj.open_list.insert(goal);
        end
        
        function Ls = successor(obj, X)
            Ls = OpenList();
            pos = [X.x; X.y];
            
            for m=obj.moves
                
                succ_pos = pos + m;
                x = succ_pos(1);
                y = succ_pos(2);

                if ~obj.map.is_inside(x, y) || obj.map.is_obstacle(x, y)
                    continue
                end

                if ~Ls.has(obj.map.map(x, y))
                    Ls = Ls.insert(obj.map.map(x, y));
                end
            end
        end

        function res = process_state(obj)
            [obj.open_list, X, Kold] = obj.open_list.pop();
            X.tag = State.TAG_CLOSED;
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
                    if Y.tag == State.TAG_NEW || ...
                            (Y.parent == X && Y.h ~= X.h + X.cost(Y)) || ...
                            (~(Y.parent == X) && Y.h > X.h + X.cost(Y))
                        Y.parent = X;
                        Y.h = X.h + X.cost(Y);
                        Y.tag = State.TAG_OPEN;
                        obj.open_list = obj.open_list.insert(Y);
                    end
                end
            else
                for i=1:size(succ.queueS, 2)
                    Y = succ.queueS(i);
                    if Y.tag == State.TAG_NEW ||...
                            (Y.parent == X && Y.h ~= X.h + X.cost(Y))
                        Y.parent = X;
                        Y.h = X.h + X.cost(Y);
                        Y.tag = State.TAG_OPEN;
                        obj.open_list = obj.open_list.insert(Y);
                    else
                        if ~(Y.parent == X) && Y.h > X.h + X.cost(Y)
                            X.tag = State.TAG_OPEN;
                            obj.open_list = obj.open_list.insert(X);
                        else
                            if ~(Y.parent == X) && X.h > Y.h + Y.cost(X) && ...
                                    Y.tag == State.TAG_CLOSED && ...
                                    Y.h > Kold
                                Y.tag = State.TAG_OPEN;
                                obj.open_list = obj.open_list.insert(Y);
                            end
                        end
                    end
                end
            end
            
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
            if state.tag == "close"
                obj.insert(state, state.parent.h + state.cost(state.parent))
            end
        end

        function run(obj, start, goal)
            PSret = 0;
            while start.tag ~= State.TAG_CLOSED && PSret ~= -1
                PSret = obj.process_state();
%                 obj.map.print_map_tag();
            end
            
            if start.tag ~= State.TAG_CLOSED && PSret == -1
                error("No possible path")
            end
    
            currPos = start;
            while ~currPos.parent.eq(goal)
                %move to minPos
                currPos = currPos.parent;
                currPos.state = Map.MAP_PATH;

                % scan graph
                % is_changed = updateMap();
                obj.map.print_map();

                % update graph
                % if is_changed
                %    update_edges_cost();
                %    compute_shortest_path();
                % end
                
            end
        end
        
    end
end