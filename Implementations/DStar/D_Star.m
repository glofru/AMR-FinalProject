classdef D_Star < handle
    properties
        globalMap;

        localMap;
        currPos;
        goal;
        moves;

        range;
        cost;

        open_list;
    end

    methods
        function obj = D_Star(globalMap, obstacles, Sstart, Sgoal,...
                moves, range, cost)
            obj.globalMap = globalMap;
            obj.moves = moves;
            obj.range = range;
            obj.cost = cost;


            % initialize map
            obj.localMap = Map(obj.globalMap.row, obj.globalMap.col, obstacles);
            
            obj.currPos = obj.localMap.map(Sstart(1), Sstart(2));
            obj.currPos.state = MapState.START;
            obj.goal = obj.localMap.map(Sgoal(1), Sgoal(2));
            obj.goal.state = MapState.GOAL;
            
            obj.open_list = OpenList(obj.goal);

            % first scan and path computation
            obj.updateMap();
            obj.computeShortestPath();
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
                    if Y.tag == StateTag.NEW || ...
                            (~isempty(Y.parent) && Y.parent == X && Y.h ~= X.h + X.cost(Y)) || ...
                            (~isempty(Y.parent) && Y.parent ~= X && Y.h > X.h + X.cost(Y))
                        Y.parent = X;
                        obj.open_list.insert(Y, X.h + X.cost(Y));
                    end
                end
            else
                for Y=succ
                    if Y.tag == StateTag.NEW ||...
                            (Y.parent == X && Y.h ~= X.h + X.cost(Y))
                        Y.parent = X;
                        obj.open_list.insert(Y, X.h + X.cost(Y));
                    else
                        if Y.parent ~= X && Y.h > X.h + X.cost(Y)
                            obj.open_list.insert(Y, X.h);
                        else
                            if Y.parent ~= X && X.h > Y.h + X.cost(Y) && ...
                                    Y.tag == StateTag.CLOSED && ...
                                    Y.h > Kold
                                obj.open_list.insert(Y, Y.h);
                            end
                        end
                    end
                end
            end

            res = obj.open_list.get_kmin();
        end

        function modify(obj)
            state = obj.currPos;
%             if state.tag == StateTag.CLOSED
                obj.open_list.insert(state, state.h + state.cost(state.parent))
%             end

            while ~obj.open_list.isEmpty()
                k_min = obj.process_state();
                if k_min >= state.h
                    break
                end
            end
        end

        function f = isFinish(obj)
            f = obj.currPos == obj.goal && ~obj.open_list.isEmpty();
        end

        function step(obj)
            obj.currPos.state = MapState.PATH;

            obj.updateMap()

            obj.localMap.plot();
            pause(0.25); % because otherwise matlab doesn't update the plot

            % update graph
            if obj.currPos.parent.state == MapState.OBSTACLE
                obj.modify()
                return
            end

            % goes forward
            obj.currPos = obj.currPos.parent;
        end

        function run(obj)
            while(~isFinish(obj))
                obj.step()
            end
        end

        function computeShortestPath(obj)
            while obj.currPos.tag ~= StateTag.CLOSED && ~obj.open_list.isEmpty()
                obj.process_state();
            end
        end

        function updateMap(obj)
            is = obj.currPos.x;
            js = obj.currPos.y;
            
            r = obj.range;

            for i=-r:r
                for j=-r:r
                    newX = is+i;
                    newY = js+j;
                    
                    if obj.localMap.isInside(newX, newY)
                        s = obj.globalMap.map(newX, newY).state;
                        if s == MapState.OBSTACLE
                            obj.localMap.map(newX, newY).state = s;
                        end
                    end
                end
            end
        end
        
    end
end