classdef D_star_lite_v1 < handle
    properties
        globalMap;
        
        localMap;
        currPos;
        goal;
        moves;
        
        U;
        obstacles;
        newObstacles;
        
        % comp data
        expCells; % for each insert +1
        expCellsList;
        totSteps; % for each cell popped from computeShortestPath +1
        totStepsList;
        pathLenght; % for each step +1
    end
    
    methods
        function obj = D_star_lite_v1(globalMap, obstacles, Sstart, Sgoal, moves, cost)
            arguments
                globalMap
                obstacles
                Sstart
                Sgoal
                moves
                
                cost = 1;
            end
            % copy vals
            obj.globalMap = globalMap;
            obj.moves = moves;
            obj.U = PriorityQueue();
            obj.obstacles = obstacles;
            obj.newObstacles = [];
            
            obj.expCells = 0;
            obj.expCellsList = [];
            obj.totSteps = 0;
            obj.totStepsList = [];
            obj.pathLenght = 0;
            
            % inizialize map
            obj.localMap = Map(obj.globalMap.row, obj.globalMap.col, [], Map.TYPE_UNKNOWN, cost);
            
            obj.currPos = obj.localMap.map(Sstart(1), Sstart(2));
            obj.currPos.state = Map.MAP_POSITION;
            obj.goal = obj.localMap.map(Sgoal(1), Sgoal(2));
            obj.goal.state = Map.MAP_GOAL;
            
            % inizialize state vals
            for i=1:obj.localMap.row
                for j=1:obj.localMap.col
                    obj.localMap.map(i, j).g = inf;
                    obj.localMap.map(i, j).rhs = inf;
                end
            end
            
            obj.goal.rhs = 0;
            obj.U = obj.U.insert(obj.goal, obj.goal.calcKey(obj.currPos));
            obj.expCells = obj.expCells+1;

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
        
        function isFin = isFinish(obj)
            if obj.currPos == obj.goal
                isFin = true;
            elseif obj.currPos.g == inf
                %disp("No possible path!");
                isFin = true;
            else
                isFin = false;
            end
        end
        
        function isChanged = updateMap(obj)
            isChanged = false;
            
            is = obj.currPos.x;
            js = obj.currPos.y;

            for i=-1:1
                for j=-1:1
                    if obj.localMap.isInside(is+i, js+j)
                        chr = obj.globalMap.map(is+i, js+j).state;
                        
                        % TODO
                        if obj.localMap.map(is+i, js+j).state ~= Map.MAP_PATH
                            obj.localMap.map(is+i, js+j).state = chr;
                        end
                            
                        if chr == Map.MAP_OBSTACLE
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
            obj.currPos.state = Map.MAP_POSITION;
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
            Lp = State.empty(1, 0);
            for m=obj.moves
                pred_pos = [u.x; u.y]+m;

                %se dentro i bordi
                if ~obj.localMap.isInside(pred_pos(1), pred_pos(2))
                    continue
                end

                isNotObs = true;
                for o=obj.obstacles
                    if all(o==pred_pos)
                        isNotObs = false;
                        break
                    end
                end

                if isNotObs
                    % TODO ottimizzare
                    pred_pos = obj.localMap.map(pred_pos(1), pred_pos(2));
                    if ~obj.isAlredyIn(Lp, pred_pos)
                        Lp(end+1) = pred_pos;
                    end
                end
            end
        end
        
        function Ls = sucessor(obj, u)
            Ls = State.empty(1, 0);
            for m=obj.moves
                pred_pos = [u.x; u.y]+m;

                %se dentro i bordi
                if ~obj.localMap.isInside(pred_pos(1), pred_pos(2))
                    continue
                end

                isNotObs = true;
                for o=obj.obstacles
                    if all(o==pred_pos)
                        isNotObs = false;
                        break
                    end
                end

                if isNotObs
                    % TODO ottimizzare
                    pred_pos = obj.localMap.map(pred_pos(1), pred_pos(2));
                    if ~obj.isAlredyIn(Ls, pred_pos)
                        Ls(end+1) = pred_pos;
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
                obj.U = obj.U.remove(u);
                obj.expCells = obj.expCells-1;
            end

            if u.g ~= u.rhs
                obj.U = obj.U.insert(u, u.calcKey(obj.currPos));
                obj.expCells = obj.expCells+1;
            end
        end
        
        function computeShortestPath(obj)
            if obj.U.isEmpty()
                    return
            end
                
            while (min2(obj.U.topKey(), obj.currPos.calcKey(obj.currPos)) || ...
                    obj.currPos.rhs ~= obj.currPos.g)
                
                obj.totSteps = obj.totSteps+1;
                
                % obj.localMap.plotMap(); % commented for fast plot
                [obj.U, u] = obj.U.pop();
                
                % TODO
                if u.state == Map.MAP_UNKNOWN || u.state == Map.MAP_EMPTY || ...
                        u.state == Map.MAP_VISITED
                    u.state = Map.MAP_START;
                end

                if (u.g > u.rhs)
                    u.g = u.rhs;
                else
                    u.g = inf;
                    obj.updateVertex(u);
                end

                pred = obj.predecessor(u);
                for p=pred
                    obj.updateVertex(p);
                end

                if obj.U.isEmpty()
                    return
                end
            end
        end

        function updateEdgesCost(obj)
            % updato tutti i predecessori degli ostacoli nuovi
            % li metto in una lista e estraggo il più vicino al goal

            updateCells = PriorityQueue();


            for o=obj.newObstacles
                oState = obj.localMap.map(o(1), o(2));

                oState.g = inf;
                oState.rhs = inf;
                pred = obj.predecessor(oState);

                for p=pred
                    if ~updateCells.has(p)
                        updateCells = updateCells.insert(p, p.calcKey(obj.currPos));
                        %obj.expCells = obj.expCells+1;
                    end
                end
            end
            obj.newObstacles = [];


            %for all directed edges (u, v)
            %    update edge cost c(u, v)
            %    updateVertex(u)
            %end

            while ~updateCells.isEmpty()
                [updateCells, s, k_old] = updateCells.pop();
                obj.updateVertex(s);
                k = s.calcKey(obj.currPos);
                if ~(k == k_old)
                    pred = obj.predecessor(s);

                    for p=pred
                        if ~updateCells.has(p)
                            updateCells = updateCells.insert(p, p.calcKey(obj.currPos));
                            %obj.expCells = obj.expCells+1;
                        end
                    end
                end
            end
            
            for s=obj.U.queue
                obj.U = obj.U.insert(s, s.calcKey(obj.currPos));
                obj.expCells = obj.expCells+1;
            end
        end
        
        function step(obj)
            % obj.totSteps = obj.totSteps+1;
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
            obj.currPos.state = Map.MAP_PATH; % TODO
            obj.currPos = minPos;

            % scan graph
            isChanged = obj.updateMap();

            % obj.localMap.plotMap();

            % update graph
            if isChanged
                % TODO optimize
                obj.updateEdgesCost();
                obj.computeShortestPath();
            end
            
            obj.expCellsList = [obj.expCellsList, obj.expCells];
            obj.totStepsList = [obj.totStepsList, obj.totSteps];
        end
        
        function run(obj)
            while(~isFinish(obj))
                obj.step()
            end
            disp("Goal reached!");
        end
    end
end