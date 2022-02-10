classdef Field_D_star < handle
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
    end
    
    methods
        % Field_D_star constructor
        function obj = Field_D_star(globalMap, obstacles, Sstart, Sgoal,...
                moves, range, cost)
            arguments
                % Map having global knowledge
                globalMap
                % set of obstacles known
                obstacles
                % start position
                Sstart
                % goal position
                Sgoal
                % set of moves that the algorithm can do
                moves
                % range of the scan
                range = 1;
                % cost of a step
                cost = 1;
            end
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
                obstacles, FDMap.TYPE_UNKNOWN, cost);
            
            obj.currPos = obj.localMap.map(Sstart(1), Sstart(2));
            obj.currPos.state = FDState.POSITION;
            obj.goal = obj.localMap.map(Sgoal(1), Sgoal(2));
            obj.goal.state = FDState.GOAL;
            
            obj.goal.rhs = 0;
            obj.OPEN.insert(obj.goal, obj.goal.calcKey(obj.currPos));
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
                    if obj.localMap.isInside(is+i, js+j)
                        chr = obj.globalMap.map(is+i, js+j).state;
                        
                        if chr == FDState.OBSTACLE
                            obj.localMap.map(is+i, js+j).state = chr;
                            
                            new_obs = [is+i, js+j];
                            if ~isAlredyIn(obj.localMap.obstacles, new_obs')
                                obj.localMap.obstacles(:, end+1) = new_obs';
                                obj.newObstacles(:, end+1) = new_obs';
                                isChanged = true;
                            end
                        end
                    end
                end
            end
            obj.currPos.state = FDState.POSITION;
        end
        
        % return the set of predecessor states of the state u
        function Lp = predecessor(obj, u)
            Lp = FDState.empty(length(obj.moves), 0);
            currI = 1;
            for m=obj.moves
                pred_pos = [u.x; u.y]+m;

                if ~obj.localMap.isInside(pred_pos(1), pred_pos(2))
                    continue
                end

                obj_pos = obj.localMap.map(pred_pos(1), pred_pos(2));
                if  obj_pos.state ~= FDState.OBSTACLE
                    % TODO ottimizzare
                    if ~isAlredyIn(Lp, obj_pos)
                        Lp(currI) = obj_pos;
                        currI = currI+1;
                    end
                end
            end
        end
        
        % return the set of successor states of the state u
        function Ls = successor(obj, u)
            Ls = FDState.empty(length(obj.moves), 0);
            currI = 1;
            for m=obj.moves
                pred_pos = [u.x; u.y]+m;

                if ~obj.localMap.isInside(pred_pos(1), pred_pos(2))
                    continue
                end

                obj_pos = obj.localMap.map(pred_pos(1), pred_pos(2));
                if obj_pos.state ~= FDState.OBSTACLE
                    % TODO ottimizzare
                    if ~isAlredyIn(Ls, obj_pos)
                        Ls(currI) = obj_pos;
                        currI = currI+1;
                    end
                end
            end
        end
        
        % update vertex u
        function updateVertex(obj, s)
            if s ~= obj.goal
                minV = inf;
                connbrs = obj.successor(s);
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

            if obj.OPEN.has(s)
                obj.OPEN.remove(s);
                obj.expCells = obj.expCells-1;
            end

            if s.g ~= s.rhs
                obj.OPEN.insert(s, s.calcKey(obj.currPos));
                obj.expCells = obj.expCells+1;
            end
        end
        
        % compute the shortest path from the goal to the current position
        function computeShortestPath(obj)
            while (min2(obj.OPEN.topKey(), obj.currPos.calcKey(obj.currPos)) || ...
                    obj.currPos.rhs ~= obj.currPos.g)
                
                obj.totSteps = obj.totSteps+1;
                
                %obj.localMap.plot(); % comment for fast plot
                %pause(0.1)
                s = obj.OPEN.pop();
                
                % TODO
                if s.state == FDState.UNKNOWN || s.state == FDState.EMPTY || ...
                        s.state == FDState.VISITED
                    s.state = FDState.START;
                end

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
                        if ~updateCells.has(p)
                            updateCells.insert(p, p.calcKey(obj.currPos));
                        end
                    end
                end
            end
        end
        
        
        % compute one step from the current position
        function step(obj)
            obj.pathLength = obj.pathLength+1;
            
            %move to minPos
            obj.currPos.state = FDState.PATH; % TODO
            [~, obj.currPos] = minVal(obj.currPos, obj.successor(obj.currPos));

            % scan graph
            isChanged = obj.updateMap();

            % update graph
            if isChanged
                % TODO optimize
                obj.updateEdgesCost();
                obj.computeShortestPath();
            end
            
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


