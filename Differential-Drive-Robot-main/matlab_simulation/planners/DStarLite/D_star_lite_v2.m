classdef D_star_lite_v2 < handle
    properties
        globalMap;
        
        localMap;
        currPos;
        goal;
        moves;
        
        U;
        obstacles;
        newObstacles;
        
        Slast;
        km;
        
        map_limit;
        map;
        start;
        size_x;
        size_y;
        resolution;
        maxIter;
    end
    
    methods
        function obj = D_star_lite_v2(init_state, sampling_time, limit, goal, map, resolution, maxIter)
            obj.map_limit = limit;
            obj.goal = int16(goal/resolution);
            obj.start = [int16(init_state(1)/resolution) int16(init_state(2)/resolution)];
            obj.resolution = resolution;
            obj.maxIter = maxIter;
            
            obj.size_x = size(map, 1);
            obj.size_y = size(map, 2);
            obj.map = zeros(obj.size_x * obj.size_y, 6);
            
            for i=1:obj.size_x
                for j=1:obj.size_y
                    if (map(i, j) < 250)
                        obj.obstacles = [obj.obstacles, [i;j]];
                    end
                end
            end
            
            D1 = obj.size_x;
            D2 = obj.size_y;
            for i=1:round(D1*D2/4)
                x = round(mod(rand*D1, D1))+1;
                y = round(mod(rand*D2, D2))+1;

                % obstacles overlap, ok, not an error
                if ~(all([x, y]==obj.start) || all([x, y]==obj.goal))
                    map(x, y) = 0;
                end
            end
            
            
            % copy vals
            obj.globalMap = map;
            obj.moves = [[1; 0], [1; 1], [0; 1], [-1; 1], [-1; 0], [-1; -1], [0; -1], [1; -1]];
            obj.U = PriorityQueue();
            obj.km = 0;
            obj.newObstacles = [];
            
            % inizialize map
            obj.localMap = DLMap(obj.size_x, obj.size_y, obj.obstacles, DLMap.TYPE_UNKNOWN);
            
            obj.currPos = obj.localMap.map(obj.start(1), obj.start(2));
            obj.currPos.state = DLMapState.POSITION;
            obj.Slast = obj.currPos;
            obj.goal = obj.localMap.map(obj.goal(1), obj.goal(2));
            obj.goal.state = DLMapState.GOAL;
            
            % inizialize state vals
            for i=1:obj.localMap.row
                for j=1:obj.localMap.col
                    obj.localMap.map(i, j).g = inf;
                    obj.localMap.map(i, j).rhs = inf;
                end
            end
            
            obj.goal.rhs = 0;
            obj.U = obj.U.insert(obj.goal, obj.goal.calcKey(obj.currPos, obj.km));

            % first scan
            obj.updateMap();
            
            %tic
            % TODO optimize
            % compute first path
            obj.computeShortestPath();
            %disp('computeShortestPath: '+string(toc)+' s'+newline);
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

            for i=-1:1
                for j=-1:1
                    if obj.localMap.isInside(is+i, js+j)
                        chr = obj.globalMap(is+i, js+j);
                            
                        if chr < 250
                            new_obs = [is+i, js+j];
                            obj.localMap.map(is+i, js+j).state = DLMapState.OBSTACLE;
                            if ~obj.isAlredyIn(obj.obstacles, new_obs')
                                obj.obstacles(:, end+1) = new_obs';
                                obj.newObstacles(:, end+1) = new_obs';
                                isChanged = true;
                            end
                        end
                    end
                end
            end
            obj.currPos.state = DLMapState.POSITION;
        end
        
        
        function Lp = predecessor(obj, u)
            Lp = DLState.empty(1, 0);
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
            Ls = DLState.empty(1, 0);
            for m=obj.moves
                pred_pos = [u.x; u.y]+m;

                % if inside boundaries
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
            end

            if u.g ~= u.rhs
                obj.U = obj.U.insert(u, u.calcKey(obj.currPos, obj.km));
            end
        end
        
        function computeShortestPath(obj)
            if obj.U.isEmpty()
                    return
            end
                
            while (min2(obj.U.topKey(), obj.currPos.calcKey(obj.currPos, obj.km)) || ...
                    obj.currPos.rhs ~= obj.currPos.g)
                obj.localMap.plotMap();
                Kold = obj.U.topKey();
                [obj.U, u] = obj.U.pop();

                % TODO
                if u.state == DLMapState.UNKNOWN || u.state == DLMapState.EMPTY || ...
                        u.state == DLMapState.VISITED
                    u.state = DLMapState.START;
                end
                                  
                
                if (Kold < u.calcKey(obj.currPos, obj.km))
                    obj.U = obj.U.insert(u, u.calcKey(obj.currPos, obj.km));
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
                [updateCells, s, k_old] = updateCells.extract(1);%pop();
                obj.updateVertex(s);
                k = s.calcKey(obj.currPos, obj.km);
                if ~(k == k_old)
                    pred = obj.predecessor(s);

                    for p=pred
                        if ~updateCells.has(p)
                            updateCells = updateCells.insert(p, p.calcKey(obj.currPos, obj.km));
                        end
                    end
                end
            end
        end
        
        function final_path = run(obj)
            final_path = ones(obj.maxIter, 6);
            dimension_path = 1;
            final_path(dimension_path, 1:2) = [obj.currPos.x, obj.currPos.y] * obj.resolution;
            
            while(obj.currPos ~= obj.goal && dimension_path < obj.maxIter)
                if obj.currPos.g == inf
                    disp("No possible path!");
                    return
                end

                minV = inf;
                minPos = DLState.empty(1, 0);
                succ = obj.sucessor(obj.currPos);
                for s=succ
                    curr = obj.currPos.c(s) + s.g;
                    if curr < minV
                        minV = curr;
                        minPos = s;
                    end
                end

                %move to minPos
                obj.currPos.state = DLMapState.PATH; % TODO 
                obj.currPos = minPos;
                dimension_path = dimension_path + 1;
                final_path(dimension_path, 1:2) = [obj.currPos.x, obj.currPos.y] * obj.resolution;
                
                % scan graph
                isChanged = obj.updateMap();
                
                obj.localMap.plotMap();

                % update graph
                if isChanged
                   obj.km = obj.km + h(obj.Slast, obj.currPos);
                   obj.Slast = obj.currPos;
                   obj.updateEdgesCost();
                   obj.computeShortestPath();
                end
            end 

            final_path = final_path(1:dimension_path, :);
            if dimension_path >= obj.maxIter
                disp("No possible path!");
            else 
                disp("Goal reached!");
            end 
        end 
    end
end