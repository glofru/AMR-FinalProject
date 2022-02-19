classdef D_star_lite_v2 < handle
    %
    %
    
    properties
        % Map having global knowledge
        globalMap;
        % Map having local knowledge
        localMap;
        % current position
        currPos;
        % goal position
        goal;
        % set of moves that the algorithm can do
        moves;
        % range of the scan
        range;
        % cost of a step
        cost;
        % priority queue
        U;
        % set of new obstacles discovered
        newObstacles;
        % previous state
        Slast;
        % km parameter
        km;
        
        % max length of the final path
        maxLengthFinalPath;
        
        % number of explored cell added to the priority queue
        expCells; % for each insert +1
        % number of explored cell added to the priority queue for each step
        expCellsList;
        % number of explored cell during computeShortestPath
        totSteps; % for each cell popped from computeShortestPath +1
        % number of explored cell during computeShortestPath for each step
        totStepsList;
        % number of steps
        pathLength; % for each step +1
    end
    
    methods
        % D_star_lite_v2 constructor
        function obj = D_star_lite_v2(globalMap, obstacles, Sstart, Sgoal,...
                moves, range, cost)
            % copy vals
            obj.globalMap = globalMap;
            obj.moves = moves;
            obj.U = DSLPriorityQueue();
            obj.km = 0;
            obj.newObstacles = [];
            obj.range = range;
            obj.cost = cost;
            
            obj.expCells = 0;
            obj.expCellsList = [];
            obj.totSteps = 0;
            obj.totStepsList = [];
            obj.pathLength = 0;
            obj.maxLengthFinalPath = (obj.globalMap.row+obj.globalMap.col)*4;
            
            % inizialize map
            obj.localMap = DSLMap(obj.globalMap.row, obj.globalMap.col,...
                obstacles, DSLMap.TYPE_UNKNOWN, cost);
            
            obj.currPos = obj.localMap.map(Sstart(1), Sstart(2));
            obj.Slast = obj.currPos;
            obj.goal = obj.localMap.map(Sgoal(1), Sgoal(2));
            
            obj.goal.rhs = 0;
            obj.U.insert(obj.goal, obj.goal.calcKey(obj.currPos, obj.km));
            obj.expCells = obj.expCells+1;

            % first scan
            obj.updateMap();
            
            % TODO optimize
            % compute first path
            obj.computeShortestPath();
            
            obj.expCellsList = [obj.expCellsList, obj.expCells];
            obj.totStepsList = [obj.totStepsList, obj.totSteps];
        end
        
        
        % check if the algorithm is finished
        function isFin = isFinish(obj)
            if obj.currPos == obj.goal
                isFin = true;
            elseif obj.currPos.g == inf
                error("Path not found")
            else
                isFin = false;
            end
        end


        % scan the map for new obstacles
        function updateMap(obj)
            is = obj.currPos.x;
            js = obj.currPos.y;
            
            r = obj.range;

            for i=-r:r
                for j=-r:r
                    newX = is+i;
                    newY = js+j;
                    if obj.localMap.isInside(newX, newY)
                        chr = obj.globalMap.map(newX, newY).state;
                        
                        state = obj.localMap.map(is+i, js+j);
                        if chr == DSLState.OBSTACLE
                            if state.state ~= chr
                                state.state = chr;
                                new_obs = [newX; newY];
                                obj.localMap.obstacles(:, end+1) = new_obs;
                                obj.newObstacles(:, end+1) = new_obs;
                            end
                        end
                    end
                end
            end
        end
        
        % return the set of predecessor states of the state u
        function Lp = predecessor(obj, u)
            Lp = DSLState.empty(length(obj.moves), 0);
            currI = 1;
            for m=obj.moves
                x = u.x + m(1);
                y = u.y + m(2);

                if obj.localMap.isInside(x, y) && ~obj.localMap.isObstacle(x, y)
                    Lp(currI) = obj.localMap.map(x, y);
                    currI = currI+1;
                end
            end
        end
        
        % return the set of successor states of the state u
        function Ls = successor(obj, u)
            Ls = DSLState.empty(length(obj.moves), 0);
            currI = 1;
            for m=obj.moves
                x = u.x + m(1);
                y = u.y + m(2);

                if obj.localMap.isInside(x, y) && ~obj.localMap.isObstacle(x, y)
                    Ls(currI) = obj.localMap.map(x, y);
                    currI = currI+1;
                end
            end
        end
        
        % update vertex u
        function updateVertex(obj, u)
            if u ~= obj.goal
                [u.rhs, ~] = minVal(u, obj.successor(u));
            end

            obj.U.removeIfPresent(u);

            if u.g ~= u.rhs
                obj.U.insert(u, u.calcKey(obj.currPos, obj.km));
                obj.expCells = obj.expCells+1;
            end
        end
        
        % compute the shortest path from the goal to the current position
        function computeShortestPath(obj)
            while true
                [u, Kold, pos] = obj.U.top();
                if ~(min2(Kold, obj.currPos.calcKey(obj.currPos, obj.km)) || ...
                    obj.currPos.rhs ~= obj.currPos.g)
                    return
                end
                
                obj.totSteps = obj.totSteps+1;
                
                obj.U.removeIndex(pos);

                if (Kold < u.calcKey(obj.currPos, obj.km))
                    obj.U.insert(u, u.calcKey(obj.currPos, obj.km));
                    obj.expCells = obj.expCells+1;
                elseif (u.g > u.rhs)
                    u.g = u.rhs;
                    pred = obj.predecessor(u);
                    for p=pred
                        obj.updateVertex(p);
                    end
                else
                    u.g = inf;
                    pred = [obj.predecessor(u), u];
                    for p=pred
                        obj.updateVertex(p);
                    end
                end
            end
        end

        % update the cost of all the cells needed when new obstacles are
        % discovered
        function updateEdgesCost(obj)
            updateCells = DSLPriorityQueue();
            updateCells.insert(obj.currPos, obj.currPos.calcKey(obj.currPos, obj.km));
            
            for o=obj.newObstacles
                oState = obj.localMap.map(o(1), o(2));
                oState.g = inf;
                oState.rhs = inf;
                oState.k = oState.calcKey(obj.currPos, obj.km);
                
                pred = obj.predecessor(oState);
                for p=pred
                    updateCells.insert(p, p.calcKey(obj.currPos, obj.km));
                end
            end
            obj.newObstacles = [];

            %for all directed edges (u, v)
            %    update edge cost c(u, v)
            %    updateVertex(u)
            %end

            while ~updateCells.isEmpty()
                [s, k_old] = updateCells.extract(1);
                obj.updateVertex(s);
                k = s.calcKey(obj.currPos, obj.km);
                if ~(k == k_old)
                    pred = obj.predecessor(s);

                    for p=pred
                        updateCells.insert(p, p.calcKey(obj.currPos, obj.km));
                    end
                end
            end
        end
        
        
        % return the set of successor states of the state u
        function Ls = successor2(obj, u)
            Ls = DSLState.empty(length(obj.moves), 0);
            currI = 1;
            for m=obj.moves
                x = u.x + m(1);
                y = u.y + m(2);

                if obj.localMap.isInside(x, y)
                    Ls(currI) = obj.localMap.map(x, y);
                    currI = currI+1;
                end
            end
        end
        
        % compute one step from the current position
        function step(obj)
            obj.pathLength = obj.pathLength+1;
            
            % move to minPos
            [~, nextState] = minVal(obj.currPos, obj.successor2(obj.currPos));

            % scan graph
            obj.updateMap();

            %obj.localMap.plot();
            %pause(0.01); % because otherwise matlab doesn't update the plot

            % update graph
            if nextState.state == DSLState.OBSTACLE
                % TODO optimize
                obj.updateEdgesCost();
                obj.computeShortestPath();
            end
            
            obj.currPos.state = DSLState.PATH; % TODO
            [~, obj.currPos] = minVal(obj.currPos, obj.successor(obj.currPos));
            
            obj.expCellsList = [obj.expCellsList, obj.expCells];
            obj.totStepsList = [obj.totStepsList, obj.totSteps];
        end
        
        % run the algorithm until reach the end
        function finalPath = run(obj)
            finalPath = zeros(obj.maxLengthFinalPath, 2);
            dimensionPath = 1;
            finalPath(dimensionPath, :) = [obj.currPos.x, obj.currPos.y];
            
            while(~isFinish(obj) && dimensionPath < obj.maxLengthFinalPath)
                obj.step()
                
                dimensionPath = dimensionPath + 1;
                finalPath(dimensionPath, :) = [obj.currPos.x, obj.currPos.y];
            end
            
            if dimensionPath >= obj.maxLengthFinalPath % never happened
                disp("loop")
                error("loop")
            end
            
            finalPath = finalPath(1:dimensionPath, :);
        end
    end
end


