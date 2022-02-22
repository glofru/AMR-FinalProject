classdef D_star_lite_v2 < handle
    % Implementation of D* lite v2 from the paper:
    % "Fast Replanning for Navigation in Unknown Terrain"
    
    properties
        % Map having global knowledge
        globalMap;
        % Map having local knowledge
        localMap;
        % Current position
        currPos;
        % Goal position
        goal;
        % Set of moves that the algorithm can do
        moves;
        % Range of the scan
        range;
        % Cost of a step
        cost;
        % Priority queue
        U;
        % Set of obstacles
        obstacles;
        % Set of new obstacles discovered
        newObstacles;

        Slast;
        km;
        
        % Utility to generate the video
        plotVideo;
    end
    
    methods
        % D_star_lite_v2 constructor
        function obj = D_star_lite_v2(globalMap, obstacles, Sstart, Sgoal,...
                moves, range, cost, plotVideo)
            arguments
                % Map having global knowledge
                globalMap
                % Set of obstacles known
                obstacles
                % Start position
                Sstart
                % Goal position
                Sgoal
                % Set of moves that the algorithm can do
                moves
                % Range of the scan
                range = 1;
                % Cost of a step
                cost = 1;
                % if true plot map
                plotVideo = 0;
            end
            
            obj.globalMap = globalMap;
            obj.moves = moves;
            obj.U = PriorityQueue();
            obj.km = 0;
            obj.newObstacles = [];
            obj.range = range;
            obj.cost = cost;
            obj.plotVideo = plotVideo;
            
            % Map initialization
            obj.localMap = Map(obj.globalMap.row, obj.globalMap.col,...
                obstacles, Map.TYPE_UNKNOWN, cost);
            
            obj.currPos = obj.localMap.map(Sstart(1), Sstart(2));
            obj.currPos.state = State.POSITION;
            obj.Slast = obj.currPos;
            obj.goal = obj.localMap.map(Sgoal(1), Sgoal(2));
            obj.goal.state = State.GOAL;
            
            obj.goal.rhs = 0;
            obj.U.insert(obj.goal, obj.goal.calcKey(obj.currPos, obj.km));

            % First scan and path computation
            obj.updateMap();
            obj.computeShortestPath();
        end
        
        % Check if the algorithm is finished
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
        
        % Scan the map for new obstacles
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
                            state = obj.localMap.map(is+i, js+j);
                            state.state = chr;

                            new_obs = [is+i; js+j];
                            if ~isAlredyIn(obj.localMap.obstacles, new_obs)
                                state.g = inf;
                                state.rhs = inf;
                                state.k = state.calcKey(obj.currPos, obj.km);
                                obj.localMap.obstacles(:, end+1) = new_obs;
                                obj.newObstacles(:, end+1) = new_obs;
                                isChanged = true;
                            end
                        end
                    end
                end
            end
            obj.currPos.state = State.POSITION;
        end
        
        % Set of predecessor states of the state u
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
                    if ~isAlredyIn(Lp, obj_pos)
                        Lp(currI) = obj_pos;
                        currI = currI+1;
                    end
                end
            end
        end
        
        % Set of successor states of the state u
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
                    if ~isAlredyIn(Ls, obj_pos)
                        Ls(currI) = obj_pos;
                        currI = currI+1;
                    end
                end
            end
        end
        
        % Update vertex u
        function updateVertex(obj, u)
            if u ~= obj.goal
                [u.rhs, ~] = minVal(u, obj.successor(u));
            end

            if obj.U.has(u)
                obj.U.remove(u);
            end

            if u.g ~= u.rhs
                obj.U.insert(u, u.calcKey(obj.currPos, obj.km));
            end
        end
        
        % Compute the shortest path from the goal to the current position
        function computeShortestPath(obj)
            while (min2(obj.U.topKey(), obj.currPos.calcKey(obj.currPos, obj.km)) || ...
                    obj.currPos.rhs ~= obj.currPos.g)
                
                [u, Kold] = obj.U.pop();
                
                if u.state == State.UNKNOWN || u.state == State.EMPTY || ...
                        u.state == State.VISITED || u.state == State.FUTUREPATH
                    u.state = State.OPEN;
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

        % Update the cost of all the cells needed when new obstacles are
        % discovered
        function updateEdgesCost(obj)
            updateCells = PriorityQueue();
            updateCells.insert(obj.currPos, obj.currPos.calcKey(obj.currPos, obj.km));
            
            for o=obj.newObstacles
                oState = obj.localMap.map(o(1), o(2));
                updateCells.insert(oState, oState.calcKey(obj.currPos, obj.km));
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
        
        
        % Takes a step from the current position
        function step(obj)
            % move to minPos
            obj.currPos.state = State.PATH;
            [~, obj.currPos] = minVal(obj.currPos, obj.successor(obj.currPos));
            obj.currPos.state = State.POSITION;
            
            % scan graph
            isChanged = obj.updateMap();
            
            % update graph
            if isChanged
               obj.km = obj.km + h(obj.Slast, obj.currPos);
               obj.Slast = obj.currPos;
               
               obj.updateEdgesCost();
               obj.computeShortestPath();
            end
            
            if obj.plotVideo
                obj.plot();
                pause(0.01);
            end
        end
        
        % Run the algorithm until it reaches the end
        function run(obj)
            while(~isFinish(obj))
                obj.step()
            end
        end
        
        
        % Switch the cells on the shortest path to the goal from oldState
        % to newState
        function switchForFuturePath(obj, oldState, newState)
            nextStep = obj.currPos;
            while ~isempty(nextStep) && nextStep.state ~= newState && ~(nextStep == obj.goal)
                if nextStep.state == oldState
                    nextStep.state = newState;
                end
                [~, nextStep] = minVal(nextStep, obj.successor(nextStep));
            end
        end
        
        % Generate the map image
        function rgbImage = buildImageMap(obj)
            obj.switchForFuturePath(State.OPEN, State.FUTUREPATH);
            
            rgbImage = obj.localMap.buildImageMap();
            
            obj.switchForFuturePath(State.FUTUREPATH, State.OPEN);
        end
        
        % Plot the map image
        function plot(obj)
            J = obj.buildImageMap();
            imshow(J);
        end
    end
end


