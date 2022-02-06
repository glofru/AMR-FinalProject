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

        % performance metrics
        expCells;
        expCellsList;
        totSteps;
        totStepsList;
        pathLength;
        maxLenghtFinalPath;
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

            % performance metrics
            obj.expCells = 1; % 1 because the goal is initially put on the open list
            obj.expCellsList = [];
            obj.totSteps = 0;
            obj.totStepsList = [];
            obj.pathLength = 0;
            obj.maxLenghtFinalPath = (obj.globalMap.row+obj.globalMap.col)*2;

            % first scan and path computation
            obj.updateMap();
            obj.computeShortestPath();

            obj.expCellsList(end+1) = obj.expCells;
            obj.totStepsList(end+1) = obj.totSteps;
        end

        function process_state(obj)
            X = obj.open_list.min_state();
            if isempty(X)
                error("Path not found")
            end
            Kold = X.k;
            obj.open_list.remove(X);
            
            %obj.localMap.plot(X); % comment for fast plot
            
            succ = obj.localMap.neighbors(X, obj.moves);
            if Kold < X.h
                for Y=succ
                    c = Y.h + Y.cost(X) * obj.cost;
                    if Y.h <= Kold && X.h > c
                        X.parent = Y;
                        X.h = c;
                    end
                end
            end

            if Kold == X.h
                for Y=succ
                    c = X.h + X.cost(Y) * obj.cost;
                    if Y.tag == StateTag.NEW || ...
                            (~isempty(Y.parent) && Y.parent == X && Y.h ~= c) || ...
                            (~isempty(Y.parent) && Y.parent ~= X && Y.h > c)
                        Y.parent = X;
                        obj.expCells = obj.expCells + 1;
                        obj.open_list.insert(Y, c);
                    end
                end
            else
                for Y=succ
                    c = X.h + X.cost(Y) * obj.cost;
                    if Y.tag == StateTag.NEW ||...
                            (Y.parent == X && Y.h ~= c)
                        Y.parent = X;
                        obj.expCells = obj.expCells + 1;
                        obj.open_list.insert(Y, c);
                    else
                        if Y.parent ~= X && Y.h > c
                            obj.expCells = obj.expCells + 1;
                            obj.open_list.insert(Y, X.h);
                        else
                            c = Y.h + Y.cost(X) * obj.cost;
                            if Y.parent ~= X && X.h > c && ...
                                    Y.tag == StateTag.CLOSED && ...
                                    Y.h > Kold
                                obj.expCells = obj.expCells + 1;
                                obj.open_list.insert(Y, Y.h);
                            end
                        end
                    end
                end
            end
        end

        function modify_cost(obj, state)
            if state.tag == StateTag.CLOSED
                obj.open_list.insert(state, state.cost(state.parent))
            end

            while ~obj.open_list.isEmpty()
                obj.process_state();
                if obj.open_list.get_kmin() >= state.h
                    break
                end
            end
        end

        function f = isFinish(obj)
            f = obj.currPos == obj.goal;
        end

        function step(obj)
            obj.currPos.state = MapState.PATH;

            obj.updateMap()

            obj.localMap.plot();
            pause(0.25); % because otherwise matlab doesn't update the plot

            % update graph
            if obj.currPos.parent.state == MapState.OBSTACLE
                obj.modify_cost(obj.currPos)
                return
            end

            % metrics
            obj.pathLength = obj.pathLength + 1;
            obj.expCellsList(end+1) = obj.expCells;
            obj.totStepsList(end+1) = obj.totSteps;

            % goes forward
            obj.currPos = obj.currPos.parent;
        end

        function finalPath = run(obj)
            finalPath = zeros(obj.maxLenghtFinalPath, 2);
            dimensionPath = 1;
            finalPath(dimensionPath, :) = [obj.currPos.x, obj.currPos.y];
            
            while(~isFinish(obj))
                obj.step()

                dimensionPath = dimensionPath + 1;
                finalPath(dimensionPath, :) = [obj.currPos.x, obj.currPos.y];
            end

            finalPath = finalPath(1:dimensionPath, :);
        end

        function computeShortestPath(obj)
            while obj.currPos.tag ~= StateTag.CLOSED && ~obj.open_list.isEmpty()
                obj.totSteps = obj.totSteps + 1;
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