classdef Field_D_star < handle
    % Implementation of Field D* from the paper:
    % "Field D*: An Interpolation-based Path Planner and Replanner"
    
    properties
        g_diff = 0;
        
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
        OPEN;
        % Set of new obstacles discovered
        newObstacles;
        
        % Utility to generate the video
        plotVideo;
        saveVideo;
        
        writerObj;
    end
    
    methods
        % Field_D_star constructor
        function obj = Field_D_star(globalMap, obstacles, Sstart, Sgoal,...
                moves, range, cost, plotVideo, saveVideo, writerObj)
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
                saveVideo = 0;
                
                writerObj = 0;
            end
            obj.globalMap = globalMap;
            obj.moves = moves;
            obj.OPEN = PriorityQueue();
            obj.newObstacles = [];
            obj.range = range;
            obj.cost = cost;
            obj.plotVideo = plotVideo;
            obj.saveVideo = saveVideo;
            
            obj.writerObj = writerObj;
            
            % Map initialization
            obj.localMap = Map(obj.globalMap.row, obj.globalMap.col,...
                obstacles, Map.TYPE_UNKNOWN, cost);
            
            obj.currPos = obj.localMap.map(Sstart(1), Sstart(2));
            obj.currPos.state = State.POSITION;
            obj.goal = obj.localMap.map(Sgoal(1), Sgoal(2));
            obj.goal.state = State.GOAL;
            
            obj.goal.rhs = 0;
            obj.OPEN.insert(obj.goal, obj.goal.calcKey(obj.currPos));

            % First scan and path computation
            obj.updateMap();
            
            % compute first path
            obj.computeShortestPath();
            
            if obj.plotVideo
                obj.plot();
                pause(0.01);
            end
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
                                state.k = state.calcKey(obj.currPos);
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
        function updateState(obj, s)
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
            end

            if s.g ~= s.rhs
                obj.OPEN.insert(s, s.calcKey(obj.currPos));
            end
        end
        
        % Compute the shortest path from the goal to the current position
        function computeShortestPath(obj)
            while (min2(obj.OPEN.topKey(), obj.currPos.calcKey(obj.currPos)) || ...
                    obj.currPos.rhs ~= obj.currPos.g)
                s = obj.OPEN.pop();
                
                if s.state == State.UNKNOWN || s.state == State.EMPTY || ...
                        s.state == State.VISITED || s.state == State.FUTUREPATH
                    s.state = State.OPEN;
                end
                
                if obj.saveVideo
                    frame = obj.buildImageMap();
                    writeVideo(obj.writerObj, frame);
                    writeVideo(obj.writerObj, frame);
                    writeVideo(obj.writerObj, frame);
                    writeVideo(obj.writerObj, frame);
                    writeVideo(obj.writerObj, frame);
                end

                if (s.g > s.rhs)
                    s.g = s.rhs;
                    pred = obj.predecessor(s);
                    for s1=pred
                        obj.updateState(s1);
                    end
                else
                    s.g = inf;
                    pred = [obj.predecessor(s), s];
                    for s1=pred
                        obj.updateState(s1);
                    end
                end
            end
        end

        % Update the cost of all the cells needed when new obstacles are
        % discovered
        function updateEdgesCost(obj)
            updateCells = PriorityQueue();
            updateCells.insert(obj.currPos, obj.currPos.calcKey(obj.currPos));
            
            for o=obj.newObstacles
                oState = obj.localMap.map(o(1), o(2));
                updateCells.insert(oState, oState.calcKey(obj.currPos));
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
                obj.updateState(s);
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
        
        
        % Takes a step from the current position
        function step(obj)
            %move to minPos
            obj.currPos.state = State.PATH;
            old_g = obj.currPos.g;
            [~, obj.currPos] = minVal(obj.currPos, obj.successor(obj.currPos));
            obj.currPos.state = State.POSITION;
            obj.g_diff = obj.g_diff + old_g - obj.currPos.g;
            
            
            
            % scan graph
            isChanged = obj.updateMap();

            % update graph
            if isChanged
                obj.updateEdgesCost();
                obj.computeShortestPath();
            end
            
            if obj.plotVideo
                obj.plot();
                pause(0.01);
            end
        end
        
        % Run the algorithm until it reaches the end
        function final_path = run(obj)
            
            final_path = obj.currPos;
            while(~isFinish(obj))
                obj.step();
                final_path(end+1) = obj.currPos;
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


