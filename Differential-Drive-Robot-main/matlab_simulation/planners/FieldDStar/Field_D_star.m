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
        U;
        % set of new obstacles discovered
        newObstacles;
        
        % map limit
        map_limit;
        % map matrix
        map;
        % initial position
        start;
        % map X size
        sizeX;
        % map X size
        sizeY;
        % resolution
        resolution;
        % max iter to find a path
        maxIter;
    end
    
    methods
        % Field_D_star constructor
        function obj = Field_D_star(init_state, sampling_time, limit, goal,...
                map, resolution, maxIter,  moves, range, cost)
            arguments
                % initial position
                init_state
                % sampling time
                sampling_time
                % map limit
                limit
                % goal position
                goal
                % map matrix
                map
                % resolution
                resolution
                % max iter to find a path
                maxIter
                
                % set of moves that the algorithm can do
                moves
                % range of the scan
                range = 1;
                % cost of a step
                cost = 1;
            end
            obj.map_limit = limit;
            obj.goal = int16(goal/resolution);
            obj.start = [int16(init_state(1)/resolution) int16(init_state(2)/resolution)];
            obj.resolution = resolution;
            obj.maxIter = maxIter;
            obj.moves = moves;
            obj.range = range;
            obj.cost = cost;
            
            obj.map = zeros(size(map,1)*size(map,2),6);
            obj.sizeX = size(map,1);
            obj.sizeY = size(map,2);
            
            obstacles = [];
            for i = 1:size(map,1)
               for j = 1:size(map,2) 
                  if(map(i,j) < 250) 
                      obstacles = [obstacles, [i; j]];
                  end
               end
            end
            
            D1 = obj.sizeX;
            D2 = obj.sizeY;
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
            obj.moves = moves;
            obj.U = PriorityQueue();
            obj.newObstacles = [];
            obj.range = range;
            obj.cost = cost;
            
            % inizialize map
            obj.localMap = Map(obj.sizeX, obj.sizeY, obstacles,...
                Map.TYPE_UNKNOWN, cost);
            
            obj.currPos = obj.localMap.map(obj.start(1), obj.start(2));
            obj.currPos.state = State.POSITION;
            obj.goal = obj.localMap.map(obj.goal(1), obj.goal(2));
            obj.goal.state = State.GOAL;
            
            obj.goal.rhs = 0;
            obj.U.insert(obj.goal, obj.goal.calcKey(obj.currPos));

            % first scan
            obj.updateMap();
            
            % TODO optimize
            % compute first path
            obj.computeShortestPath();
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
                        chr = obj.globalMap(is+i, js+j);
                        
                        if chr < 250 % == Map.MAP_OBSTACLE
                            obj.localMap.map(is+i, js+j).state = State.OBSTACLE;
                            
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
            obj.currPos.state = State.POSITION;
        end
        
        % return the set of predecessor states of the state u
        function Lp = predecessor(obj, u)
            Lp = State.empty(length(obj.moves), 0);
            currI = 1;
            for m=obj.moves
                pred_pos = [u.x; u.y]+m;

                if ~obj.localMap.isInside(pred_pos(1), pred_pos(2))
                    continue
                end

                obj_pos = obj.localMap.map(pred_pos(1), pred_pos(2));
                if  obj_pos.state ~= State.OBSTACLE
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
            Ls = State.empty(length(obj.moves), 0);
            currI = 1;
            for m=obj.moves
                pred_pos = [u.x; u.y]+m;

                if ~obj.localMap.isInside(pred_pos(1), pred_pos(2))
                    continue
                end

                obj_pos = obj.localMap.map(pred_pos(1), pred_pos(2));
                if obj_pos.state ~= State.OBSTACLE
                    % TODO ottimizzare
                    if ~isAlredyIn(Ls, obj_pos)
                        Ls(currI) = obj_pos;
                        currI = currI+1;
                    end
                end
            end
        end
        
        % s, sa, sb are neighbourds
        % c is the traversal cost of the center cell
        % b is the traversal cost of the bottom cell
        function vs = computeCost(obj, s, sa, sb)
            if (s.x ~= sa.x && s.y ~= sa.y)
                s1 = sa;
                s2 = sb;
            else
                s1 = sb;
                s2 = sa;
            end
            
            c = 0.5;
            b = 1;
            
            if (min(c,b) == inf)
                vs = inf;
            elseif (s1.g <= s2.g)
                vs = min(c, b) + s1.g;
            else
                f = s1.g - s2.g;
                
                if (f <= b)
                    if (c <= f)
                        vs = c*sqrt(2) + s2.g;
                    else
                        y = min(f/(sqrt(c^2-f^2)), 1);
                        vs = c*sqrt(1+y^2)+f*(1-y)+s2.g;
                    end
                else
                    if (c <= b)
                        vs = c*sqrt(2)+s2.g;
                    else
                        x = 1-min(b/(sqrt(c^2-b^2)), 1);
                        vs = c*sqrt(1+(1-x)^2)+b*x+s2.g;
                    end
                end
            end
        end
        
        % update vertex u
        function updateVertex(obj, u)
            if u ~= obj.goal
                minV = inf;
                 connbrs = obj.successor(u);
                for i=[1:length(connbrs); 2:length(connbrs), 1]
                    
                    s1 = connbrs(i(1));
                    s2 = connbrs(i(2));
                    curr = obj.computeCost(u, s1, s2);
                    if curr < minV
                        minV = curr;
                    end
                end
                u.rhs = minV;
            end

            if obj.U.has(u)
                obj.U.remove(u);
            end

            if u.g ~= u.rhs
                obj.U.insert(u, u.calcKey(obj.currPos));
            end
        end
        
        % compute the shortest path from the goal to the current position
        function computeShortestPath(obj)
            while (min2(obj.U.topKey(), obj.currPos.calcKey(obj.currPos)) || ...
                    obj.currPos.rhs ~= obj.currPos.g)
                obj.localMap.plot();
                %pause(0.1)
                u = obj.U.pop();
                
                % TODO
                if u.state == State.UNKNOWN || u.state == State.EMPTY || ...
                        u.state == State.VISITED
                    u.state = State.START;
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
            updateCells = PriorityQueue();
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
                [s, k_old] = updateCells.extract(1);%pop();
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
        
        % run the algorithm until reach the end
        function final_path = run(obj)
            final_path = ones(obj.maxIter,6);
            dimension_path = 1;
            final_path(dimension_path,1:2) = [obj.currPos.x, obj.currPos.y] * obj.resolution; 
            
            while(obj.currPos ~= obj.goal && dimension_path < obj.maxIter)
                if obj.currPos.g == inf
                    disp("No possible path!");
                    return
                end

                % move to minPos
                obj.currPos.state = State.PATH; % TODO
                [~, obj.currPos] = minVal(obj.currPos, obj.successor(obj.currPos));
                
                dimension_path = dimension_path + 1;
                final_path(dimension_path,1:2) = [obj.currPos.x, obj.currPos.y] * obj.resolution; 

                % scan graph
                isChanged = obj.updateMap();
                
                obj.localMap.plot();

                % update graph
                if isChanged
                    % TODO optimize
                    obj.updateEdgesCost();
                    obj.computeShortestPath();
                end

            end
            
            final_path = final_path(1:dimension_path,:);
            
            if dimension_path >= obj.maxIter
                disp("No possible path!");
                return
            else
                disp("Goal reached!");
            end
        end
    end
end


