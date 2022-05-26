classdef Field_D_star < handle
    % Optimized implementation of Field D* from the paper:
    % "Field D*: An Interpolation-based Path Planner and Replanner"
    
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
        OPEN;
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
        % replanning time
        replanningTime = 0;
        % number of replanning occurencies
        replanningOccurencies = 0;
    end
    
    methods
        % Field_D_star constructor
        function obj = Field_D_star(globalMap, obstacles, Sstart, Sgoal,...
                moves, range, cost)
            % copy vals
            obj.globalMap = globalMap;
            obj.moves = moves;
            obj.OPEN = FDPriorityQueue();
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
            obj.localMap = FDMap(obj.globalMap.row, obj.globalMap.col,...
                obstacles, cost);
            
            obj.currPos = obj.localMap.map(Sstart(1), Sstart(2));
            obj.currPos.setPos(Sstart(1), Sstart(2), obj.cost);
            obj.goal = obj.localMap.map(Sgoal(1), Sgoal(2));
            obj.goal.setPos(Sgoal(1), Sgoal(2), obj.cost);
            
            obj.goal.rhs = 0;
            obj.OPEN.insert(obj.goal, obj.goal.calcKey(obj.currPos));
            obj.expCells = obj.expCells+1;

            % first scan
            obj.updateMap();
            
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
                        
                        state = obj.localMap.map(newX, newY);
                        if chr == FDState.OBSTACLE
                            if state.state ~= chr
                                state.state = chr;
                                state.k = state.calcKey(obj.currPos);
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
            Lp = FDState.empty(length(obj.moves), 0);
            currI = 1;
            for m=obj.moves
                x = u.x + m(1);
                y = u.y + m(2);

                if obj.localMap.isInside(x, y) && ~obj.localMap.isObstacle(x, y)
                    state = obj.localMap.map(x, y);
                    if isempty(state.cost)
                        state.setPos(x, y, obj.cost);
                        state.state = DSLState.VISITED;
                    end
                    Lp(currI) = state;
                    currI = currI+1;
                end
            end
        end
        
        % return the set of successor states of the state u
        function Ls = successor(obj, u)
            Ls = FDState.empty(length(obj.moves), 0);
            currI = 1;
            for m=obj.moves
                x = u.x + m(1);
                y = u.y + m(2);

                if obj.localMap.isInside(x, y) && ~obj.localMap.isObstacle(x, y)
                    state = obj.localMap.map(x, y);
                    if isempty(state.cost)
                        state.setPos(x, y, obj.cost);
                        state.state = DSLState.VISITED;
                    end
                    Ls(currI) = state;
                    currI = currI+1;
                end
            end
        end
        
        % update vertex u
        function updateVertex(obj, s)
            if s ~= obj.goal
                minV = inf;
                connbrs = obj.successor(s);
                if isempty(connbrs)
                    s.rhs = Inf;
                else
                    for i=[1:length(connbrs); 2:length(connbrs), 1]
                        s1 = connbrs(i(1));
                        s2 = connbrs(i(2));
                        curr = s.computeCost(s1, s2);
                        if curr < minV
                            minV = curr;
                        end
                    end
                    s.rhs = minV;
                end
            end

            obj.OPEN.removeIfPresent(s);

            if s.g ~= s.rhs
                obj.OPEN.insert(s, s.calcKey(obj.currPos));
                obj.expCells = obj.expCells+1;
            end
        end
        
        % compute the shortest path from the goal to the current position
        function computeShortestPath(obj)
            while true
                [s, k, pos] = obj.OPEN.top();
                if ~(min2(k, obj.currPos.calcKey(obj.currPos)) || ...
                    obj.currPos.rhs ~= obj.currPos.g)
                    return
                end
                
                obj.totSteps = obj.totSteps+1;
                
                obj.OPEN.removeIndex(pos);

                if (s.g > s.rhs)
                    s.g = s.rhs;
                    pred = obj.predecessor(s);
                    for s1=pred
                        obj.updateVertex(s1);
                    end
                else
                    s.g = inf;
                    pred = [obj.predecessor(s), s];
                    for s1=pred
                        obj.updateVertex(s1);
                    end
                end
            end
        end

        % update the cost of all the cells needed when new obstacles are
        % discovered
        function updateEdgesCost(obj)
            updateCells = FDPriorityQueue();
            updateCells.insert(obj.currPos, obj.currPos.calcKey(obj.currPos));

            for o=obj.newObstacles
                oState = obj.localMap.map(o(1), o(2));
                if isempty(oState.cost)
                    oState.setPos(o(1), o(2), obj.cost);
                end
                
                pred = obj.predecessor(oState);
                for p=pred
                    updateCells.insert(p, p.calcKey(obj.currPos));
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
                        updateCells.insert(p, p.calcKey(obj.currPos));
                    end
                end
            end
        end
        
        
        % return the set of successor states of the state u
        function Ls = successor2(obj, u)
            Ls = FDState.empty(length(obj.moves), 0);
            currI = 1;
            for m=obj.moves
                x = u.x + m(1);
                y = u.y + m(2);

                if obj.localMap.isInside(x, y)
                    state = obj.localMap.map(x, y);
                    if isempty(state.cost)
                        state.setPos(x, y, obj.cost);
                        state.state = DSLState.VISITED;
                    end
                    Ls(currI) = state;
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
            if nextState.state == FDState.OBSTACLE
                tic

                obj.updateEdgesCost();
                obj.computeShortestPath();
                
                [~, nextState] = minVal(obj.currPos, obj.successor(obj.currPos));

                obj.replanningTime = obj.replanningTime + toc;
                obj.replanningOccurencies = obj.replanningOccurencies + 1;
            end
            obj.currPos = nextState;
            obj.currPos.state = FDState.PATH;
            
            obj.expCellsList = [obj.expCellsList, obj.expCells];
            obj.totStepsList = [obj.totStepsList, obj.totSteps];
        end
        
        % run the algorithm until reach the end
        function [finalPath, averageReplanningTime] = run(obj)
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
            averageReplanningTime = obj.replanningTime / obj.replanningOccurencies;
        end
    end
end


