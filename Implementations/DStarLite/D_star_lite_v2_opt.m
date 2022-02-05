classdef D_star_lite_v2_opt < handle
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
    end
    
    methods
        function obj = D_star_lite_v2_opt(globalMap, obstacles, Sstart, Sgoal,...
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
            
            obj.localMap = Map(obj.globalMap.row, obj.globalMap.col,...
                obj.obstacles, Map.TYPE_UNKNOWN, cost);
            
            obj.currPos = obj.localMap.map(Sstart(1), Sstart(2));
            obj.currPos.state = Map.MAP_POSITION;
            obj.Slast = obj.currPos;
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
            
            %*********************OPTIMIZATION********************
            obj.U = obj.U.insert(obj.goal, [obj.currPos.h(obj.goal), 0]);

            % first scan
            obj.updateMap();
            
            % TODO optimize
            % compute first path
            obj.computeShortestPath();
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
        function [minV, minPos] = minVal(obj, u, list) % TODO
            minV = inf;
            minPos = State.empty(1, 0);
            for s=list
                curr = u.c(s) + s.g;
                if curr < minV
                    minV = curr;
                    minPos = s;
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
                            
                        if chr == Map.MAP_OBSTACLE
                            obj.localMap.map(is+i, js+j).state = chr;
                            
                            new_obs = [is+i, js+j];
                            obj.localMap.map(is+i, js+j).state = Map.MAP_OBSTACLE;
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
        
        function isFin = isFinish(obj)
            if obj.currPos == obj.goal
                disp("Goal reached!");
                isFin = true;
            elseif obj.currPos.g == inf
                disp("No possible path!");
                isFin = true;
            else
                isFin = false;
            end
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
                if  obj_pos.state ~= Map.MAP_OBSTACLE
                    % TODO ottimizzare
                    if ~obj.isAlredyIn(Lp, obj_pos)
                        Lp(currI) = obj_pos;
                        currI = currI+1;
                    end
                end
            end
        end
        
        function Ls = successor(obj, u)
            Ls = State.empty(length(obj.moves), 0);
            currI = 1;
            for m=obj.moves
                pred_pos = [u.x; u.y]+m;

                %se dentro i bordi
                if ~obj.localMap.isInside(pred_pos(1), pred_pos(2))
                    continue
                end

                obj_pos = obj.localMap.map(pred_pos(1), pred_pos(2));
                if obj_pos.state ~= Map.MAP_OBSTACLE
                    % TODO ottimizzare
                    if ~obj.isAlredyIn(Ls, obj_pos)
                        Ls(currI) = obj_pos;
                        currI = currI+1;
                    end
                end
            end
        end
        
        %*******************OPTIMIZATION*******************
        function updateVertex(obj, u)
            if u.g ~= u.rhs
                obj.U = obj.U.insert(u, u.calcKey(obj.currPos, obj.km));
            elseif u.g == u.rhs && obj.U.has(u)
                obj.U = obj.U.remove(u);
            end
        end
        %*******************FINISH OPTIMIZATION*************
        
        %*******************OPTIMIZATION*******************
        function computeShortestPath(obj)
            if obj.U.isEmpty() % added check, oth matlab error
                    return
            end
            
            while (min2(obj.U.topKey(), obj.currPos.calcKey(obj.currPos, obj.km)) || ...
                    obj.currPos.rhs ~= obj.currPos.g)
                %obj.localMap.plot(); % comment for fast plot
                [u, Kold] = obj.U.top();
                Knew = u.calcKey(obj.currPos, obj.km);
                
                % TODO
                if u.state == Map.MAP_UNKNOWN || u.state == Map.MAP_EMPTY || ...
                        u.state == Map.MAP_VISITED
                    u.state = Map.MAP_START;
                end
                
                if Kold < Knew
                    obj.U = obj.U.insert(u, Knew);
                elseif (u.g > u.rhs)
                    u.g = u.rhs;
                    obj.U = obj.U.remove(u);
                    
                    for p=obj.predecessor(u)
                        p.rhs = min(p.rhs, u.c(p) + u.g);
                        obj.updateVertex(p);
                    end
                else
                    Gold = u.g;
                    u.g = inf;
                    for p=[obj.predecessor(u), u]
                        if p.rhs == u.c(p) + Gold
                            if p ~= obj.currPos
                                [p.rhs, ~] = obj.minVal(p, obj.successor(p));
                            end
                        end
                        obj.updateVertex(p);
                    end
                end
                    
                if obj.U.isEmpty() % added check, oth matlab error
                    return
                end
            end
        end
         %*******************FINISH OPTIMIZATION*************

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
                        updateCells = updateCells.insert(p, p.calcKey(obj.currPos, obj.km));
                    end
                end
            end
            obj.newObstacles = [];


            %for all directed edges (u, v)
            %    update edge cost c(u, v)
            %    updateVertex(u)
            %end

            while ~updateCells.isEmpty()
                [updateCells, u, k_old] = updateCells.pop();
                Cold = u.c(obj.currPos);
                obj.updateVertex(u);
                k = u.calcKey(obj.currPos, obj.km);
                
                if Cold > u.c(obj.currPos)
                    u.rhs = min(u.rhs, u.c(obj.currPos) + obj.currPos.g);
                elseif u.rhs == Cold + obj.currPos.g
                    if u ~= obj.goal
                        [u.rhs, ~] = obj.minVal(u, obj.successor(u));
                    end
                end
                
                
                
                if ~(k == k_old)
                    pred = obj.predecessor(u);

                    for p=pred
                        if ~updateCells.has(p)
                            updateCells = updateCells.insert(p, p.calcKey(obj.currPos, obj.km));
                        end
                    end
                end
            end
            
%             for s=obj.U.queue
%                 obj.U = obj.U.insert(s, s.calcKey(obj.currPos, obj.km));
%             end
        end
        
        function step(obj)

            %move to minPos
            obj.currPos.state = Map.MAP_PATH; % TODO
            [~, obj.currPos] = obj.minVal(obj.currPos, obj.successor(obj.currPos));

            % scan graph
            isChanged = obj.updateMap();

            obj.localMap.plot();
            pause(0.25); % because otherwise matlab doesn't update the plot

            % update graph
            if isChanged
               obj.km = obj.km + h(obj.Slast, obj.currPos);
               obj.Slast = obj.currPos;

               obj.updateEdgesCost();
               obj.computeShortestPath();
            end
        end
        
        function run(obj)
            while(~isFinish(obj))
                obj.step()
            end
        end
    end
end


