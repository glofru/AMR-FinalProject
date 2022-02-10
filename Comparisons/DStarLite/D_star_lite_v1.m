classdef D_star_lite_v1 < handle
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
        % D_star_lite_v1 constructor
        function obj = D_star_lite_v1(globalMap, obstacles, Sstart, Sgoal,...
                moves, range, cost)
            % copy vals
            obj.globalMap = globalMap;
            obj.moves = moves;
            obj.U = DSLPriorityQueue();
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
            obj.currPos.state = DSLState.POSITION;
            obj.goal = obj.localMap.map(Sgoal(1), Sgoal(2));
            obj.goal.state = DSLState.GOAL;
            
            obj.goal.rhs = 0;
            obj.U.insert(obj.goal, obj.goal.calcKey(obj.currPos));
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
                isFin = true;
            else
                isFin = false;
            end
        end
        
        
        % scan the map for new obstacles
        function isChanged = updateMap(obj)
            isChanged = false;
            
            is = obj.currPos.x;
            js = obj.currPos.y;
            
            r = obj.range;

            for i=-r:r
                for j=-r:r
                    newX = is+i;
                    newY = js+j;
                    if obj.localMap.isInside(newX, newY)
                        chr = obj.globalMap.map(newX, newY).state;
                            
                        if chr == DSLState.OBSTACLE
                            if obj.localMap.map(newX, newY).state ~= chr
                                obj.localMap.map(newX, newY).state = chr;
                                new_obs = [newX, newY]';
                                obj.localMap.obstacles(:, end+1) = new_obs;
                                obj.newObstacles(:, end+1) = new_obs;
                                isChanged = true;
                            end
                        end
                    end
                end
            end
            obj.currPos.state = DSLState.POSITION;
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

            if obj.U.has(u)
                obj.U.remove(u);
            end

            if u.g ~= u.rhs
                obj.U.insert(u, u.calcKey(obj.currPos));
                obj.expCells = obj.expCells+1;
            end
        end
        
        % compute the shortest path from the goal to the current position
        function computeShortestPath(obj)
            while true
                [u, k] = obj.U.top();
                if ~(min2(k, obj.currPos.calcKey(obj.currPos)) || ...
                    obj.currPos.rhs ~= obj.currPos.g)
                    return
                end
                
                obj.totSteps = obj.totSteps+1;
                
                %obj.localMap.plot(); % commented for fast plot
                %pause(0.1);
                
                obj.U.remove(u);
                
                % TODO
                if u.state == DSLState.UNKNOWN || u.state == DSLState.EMPTY || ...
                        u.state == DSLState.VISITED
                    u.state = DSLState.START;
                end

                if (u.g > u.rhs)
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
            for o=obj.newObstacles
                oState = obj.localMap.map(o(1), o(2));

                oState.g = inf;
                oState.rhs = inf;
                pred = obj.predecessor(oState);

                for p=pred
                    if ~updateCells.has(p)
                        updateCells.insert(p, p.calcKey(obj.currPos));
                    end
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
                k = s.calcKey(obj.currPos);
                if ~(k == k_old)
                    pred = obj.predecessor(s);

                    for p=pred
                        %if ~updateCells.has(p)
                            updateCells.insert(p, p.calcKey(obj.currPos));
                        %end
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
            obj.currPos.state = DSLState.PATH; % TODO
            [~, nextState] = minVal(obj.currPos, obj.successor2(obj.currPos));

            % scan graph
            isChanged = obj.updateMap();

            %obj.localMap.plot();
            %pause(0.25); % because otherwise matlab doesn't update the plot

            % update graph
            if nextState.state == DSLState.OBSTACLE %isChanged
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
            
            finalPath = finalPath(1:dimensionPath, :);
        end
    end
end


