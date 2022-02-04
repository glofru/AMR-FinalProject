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
        
        Slast;
        % km parameter
        km;
    end
    
    methods
        function obj = D_star_lite_v2(globalMap, obstacles, Sstart, Sgoal,...
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
            obj.U = PriorityQueue();
            obj.km = 0;
            obj.newObstacles = [];
            obj.range = range;
            obj.cost = cost;
            
            % inizialize map
            obj.localMap = Map(obj.globalMap.row, obj.globalMap.col,...
                obstacles, Map.TYPE_UNKNOWN, cost);
            
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
        end
        
        % check if the algorithm is finished
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
                            
                        if chr == State.OBSTACLE
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
        function Ls = sucessor(obj, u)
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
        
        % update vertex u
        function updateVertex(obj, u)
            if u ~= obj.goal
                [u.rhs, ~] = minVal(u, obj.sucessor(u));
            end

            if obj.U.has(u)
                obj.U.remove(u);
            end

            if u.g ~= u.rhs
                obj.U.insert(u, u.calcKey(obj.currPos, obj.km));
            end
        end
        
        % compute the shortest path from the goal to the current position
        function computeShortestPath(obj)
            while (min2(obj.U.topKey(), obj.currPos.calcKey(obj.currPos, obj.km)) || ...
                    obj.currPos.rhs ~= obj.currPos.g)
                
                %obj.localMap.plot(); % comment for fast plot
                %pause(0.1)
                [u, Kold] = obj.U.pop();
                
                % TODO
                if u.state == State.UNKNOWN || u.state == State.EMPTY || ...
                        u.state == State.VISITED
                    u.state = State.START;
                end

                if (Kold < u.calcKey(obj.currPos, obj.km))
                    obj.U.insert(u, u.calcKey(obj.currPos, obj.km));
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
                [s, k_old] = updateCells.extract(1);
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
        
        
        % compute one step from the current position
        function step(obj)
            %move to minPos
            obj.currPos.state = State.PATH; % TODO
            [~, obj.currPos] = minVal(obj.currPos, obj.sucessor(obj.currPos));

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
        
        % run the algorithm until reach the end
        function run(obj)
            while(~isFinish(obj))
                obj.step()
            end
        end
    end
end