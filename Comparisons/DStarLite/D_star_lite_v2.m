classdef D_star_lite_v2 < handle
    properties
        globalMap;
        
        localMap;
        currPos;
        goal;
        moves;
        range;
        cost;
        
        U;
        obstacles;
        newObstacles;
        
        Slast;
        km;
        
        % comp data
        expCells; % for each insert +1
        expCellsList;
        totSteps; % for each cell popped from computeShortestPath +1
        totStepsList;
        pathLenght; % for each step +1
        maxLenghtFinalPath;
    end
    
    methods
        function obj = D_star_lite_v2(globalMap, obstacles, Sstart, Sgoal,...
                moves, range, cost)
            arguments
                globalMap
                obstacles
                Sstart
                Sgoal
                moves
                
                range = 1;
                cost = 1;
            end
            % copy vals
            obj.globalMap = globalMap;
            obj.moves = moves;
            obj.U = PriorityQueue();
            obj.km = 0;
            obj.obstacles = obstacles;
            obj.newObstacles = [];
            obj.range = range;
            obj.cost = cost;
            
            obj.expCells = 0;
            obj.expCellsList = [];
            obj.totSteps = 0;
            obj.totStepsList = [];
            obj.pathLenght = 0;
            obj.maxLenghtFinalPath = (obj.globalMap.row+obj.globalMap.col)*2;
            
            % inizialize map
            obj.localMap = Map(obj.globalMap.row, obj.globalMap.col,...
                obj.obstacles, Map.TYPE_UNKNOWN, cost);
            
            obj.currPos = obj.localMap.map(Sstart(1), Sstart(2));
            obj.currPos.state = State.POSITION;
            obj.Slast = obj.currPos;
            obj.goal = obj.localMap.map(Sgoal(1), Sgoal(2));
            obj.goal.state = State.GOAL;
            
            obj.goal.rhs = 0;
            obj.U.insert(obj.goal, obj.goal.calcKey(obj.currPos, obj.km));

            % first scan
            obj.updateMap();
            
            % TODO optimize
            % compute first path
            obj.computeShortestPath();
            
            obj.expCellsList = [obj.expCellsList, obj.expCells];
            obj.totStepsList = [obj.totStepsList, obj.totSteps];
        end
        
        function isIn = isAlredyIn(obj, L, val) % TODO
            % check if val is inside list L

            isIn = false;
            for elem=L
                if all(elem==val)
                    isIn = true;
                    break
                end
            end
        end

        function isChanged = updateMap(obj)
            isChanged = false;
            
            is = obj.currPos.x;
            js = obj.currPos.y;
            
            r = obj.range;

            for i=-r:r
                for j=-r:r
                    if obj.localMap.isInside(is+i, js+j)
                        chr = obj.globalMap.map(is+i, js+j).state;
                            
                        if chr == State.OBSTACLE
                            obj.localMap.map(is+i, js+j).state = chr;

                            new_obs = [is+i, js+j];
                            if ~obj.isAlredyIn(obj.obstacles, new_obs')
                                obj.obstacles(:, end+1) = new_obs';
                                obj.newObstacles(:, end+1) = new_obs';
                                isChanged = true;
                            end
                        end
                    end
                end
            end
            obj.currPos.state = State.POSITION;
        end
        
        function isFin = isFinish(obj)
            if obj.currPos == obj.goal
                isFin = true;
            elseif obj.currPos.g == inf
                isFin = true;
            else
                isFin = false;
            end
        end
        
        function ret = getExpCells(obj)
            ret = obj.expCells;
        end
        
        function ret = getTotSteps(obj)
            ret = obj.totSteps;
        end
        
        function ret = getPathLenght(obj)
            ret = obj.pathLenght;
        end
        
        
        function Lp = predecessor(obj, u)
            Lp = State.empty(length(obj.moves), 0);
            currI = 1;
            for m=obj.moves
                pred_pos = [u.x; u.y]+m;

                %se dentro i bordi
                if ~obj.localMap.isInside(pred_pos(1), pred_pos(2))
                    continue
                end

                obj_pos = obj.localMap.map(pred_pos(1), pred_pos(2));
                if  obj_pos.state ~= State.OBSTACLE
                    % TODO ottimizzare
                    if ~obj.isAlredyIn(Lp, obj_pos)
                        Lp(currI) = obj_pos;
                        currI = currI+1;
                    end
                end
            end
        end
        
        function Ls = sucessor(obj, u)
            Ls = State.empty(length(obj.moves), 0);
            currI = 1;
            for m=obj.moves
                pred_pos = [u.x; u.y]+m;

                %se dentro i bordi
                if ~obj.localMap.isInside(pred_pos(1), pred_pos(2))
                    continue
                end

                obj_pos = obj.localMap.map(pred_pos(1), pred_pos(2));
                if obj_pos.state ~= State.OBSTACLE
                    % TODO ottimizzare
                    if ~obj.isAlredyIn(Ls, obj_pos)
                        Ls(currI) = obj_pos;
                        currI = currI+1;
                    end
                end
            end
        end
        
        function updateVertex(obj, u)
            if u ~= obj.goal
                minV = inf;
                succ = obj.sucessor(u);
                for s=succ
                    curr = u.c(s) + s.g;
                    if curr < minV
                        minV = curr;
                    end
                end
                u.rhs = minV;
            end

            if obj.U.has(u)
                obj.U.remove(u);
                obj.expCells = obj.expCells-1;
            end

            if u.g ~= u.rhs
                obj.U.insert(u, u.calcKey(obj.currPos, obj.km));
                obj.expCells = obj.expCells+1;
            end
        end
        
        function computeShortestPath(obj)
            while (min2(obj.U.topKey(), obj.currPos.calcKey(obj.currPos, obj.km)) || ...
                    obj.currPos.rhs ~= obj.currPos.g)
                
                obj.totSteps = obj.totSteps+1;
                
                %obj.localMap.plot(); % comment for fast plot
                [u, Kold] = obj.U.pop();
                
                % TODO
                if u.state == State.UNKNOWN || u.state == State.EMPTY || ...
                        u.state == State.VISITED
                    u.state = State.START;
                end

                if (Kold < u.calcKey(obj.currPos, obj.km))
                    obj.U.insert(u, u.calcKey(obj.currPos, obj.km));
                    obj.expCells = obj.expCells+1;
                elseif (u.g > u.rhs)
                    u.g = u.rhs;
                else
                    u.g = inf;
                    obj.updateVertex(u);
                end

                pred = obj.predecessor(u);
                for p=pred
                    obj.updateVertex(p);
                end
            end
        end

        function updateEdgesCost(obj)
            updateCells = PriorityQueue();
            for o=obj.newObstacles
                oState = obj.localMap.map(o(1), o(2));

                oState.g = inf;
                oState.rhs = inf;
                pred = obj.predecessor(oState);

                for p=pred
                    if ~updateCells.has(p)
                        updateCells.insert(p, p.calcKey(obj.currPos, obj.km));
                    end
                end
            end
            obj.newObstacles = [];

            %for all directed edges (u, v)
            %    update edge cost c(u, v)
            %    updateVertex(u)
            %end

            while ~updateCells.isEmpty()
                [s, k_old] = updateCells.pop();
                obj.updateVertex(s);
                k = s.calcKey(obj.currPos, obj.km);
                if ~(k == k_old)
                    pred = obj.predecessor(s);

                    for p=pred
                        if ~updateCells.has(p)
                            updateCells.insert(p, p.calcKey(obj.currPos, obj.km));
                        end
                    end
                end
            end
        end
        
        function step(obj)
            obj.pathLenght = obj.pathLenght+1;
            
            minV = inf;
            minPos = State.empty(1, 0);
            succ = obj.sucessor(obj.currPos);
            for s=succ
                curr = obj.currPos.c(s) + s.g;
                if curr < minV
                    minV = curr;
                    minPos = s;
                end
            end

            %move to minPos
            obj.currPos.state = State.PATH; % TODO
            obj.currPos = minPos;

            % scan graph
            isChanged = obj.updateMap();

            % update graph
            if isChanged
               obj.km = obj.km + h(obj.Slast, obj.currPos);
               obj.Slast = obj.currPos;
               obj.updateEdgesCost();
               obj.computeShortestPath();
            end
            
            obj.expCellsList = [obj.expCellsList, obj.expCells];
            obj.totStepsList = [obj.totStepsList, obj.totSteps];
        end
        
        function finalPath = run(obj)
            finalPath = zeros(obj.maxLenghtFinalPath, 2);
            dimensionPath = 1;
            finalPath(dimensionPath, :) = [obj.currPos.x, obj.currPos.y];
            
            while(~isFinish(obj) && dimensionPath < obj.maxLenghtFinalPath)
                obj.step()
                
                dimensionPath = dimensionPath + 1;
                finalPath(dimensionPath, :) = [obj.currPos.x, obj.currPos.y];
            end
            
            finalPath = finalPath(1:dimensionPath, :);
        end
    end
end