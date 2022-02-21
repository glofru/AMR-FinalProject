classdef Field_D_star_opt < handle
    % Optimized implementation of Field D* from the paper:
    % "Field D*: An Interpolation-based Path Planner and Replanner"
    
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
        OPEN;
        % Set of new obstacles discovered
        newObstacles;
        
        % Utility to generate the video
        plotVideo;
    end
    
    methods
        % Field_D_star_opt constructor
        function obj = Field_D_star_opt(globalMap, obstacles, Sstart, Sgoal,...
                moves, range, cost, plotVideo)
            % copy vals
            obj.globalMap = globalMap;
            obj.moves = moves;
            obj.OPEN = PriorityQueue();
            obj.newObstacles = [];
            obj.range = range;
            obj.cost = cost;
            obj.plotVideo = plotVideo;
            
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
                    newX = is+i;
                    newY = js+j;
                    if obj.localMap.isInside(newX, newY)
                        chr = obj.globalMap.map(newX, newY).state;
                        
                        state = obj.localMap.map(newX, newY);
                        if chr == State.OBSTACLE
                            if state.state ~= chr
                                state.state = chr;
                                state.g = inf;
                                state.rhs = inf;
                                state.k = state.calcKey(obj.currPos, obj.km);
                                new_obs = [newX; newY];
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
                x = u.x + m(1);
                y = u.y + m(2);

                if obj.localMap.isInside(x, y) && ~obj.localMap.isObstacle(x, y)
                    Lp(currI) = obj.localMap.map(x, y);
                    currI = currI+1;
                end
            end
        end
        
        % Set of successor states of the state u
        function Ls = successor(obj, u)
            Ls = State.empty(length(obj.moves), 0);
            currI = 1;
            for m=obj.moves
                x = u.x + m(1);
                y = u.y + m(2);

                if obj.localMap.isInside(x, y) && ~obj.localMap.isObstacle(x, y)
                    Ls(currI) = obj.localMap.map(x, y);
                    currI = currI+1;
                end
            end
        end
        
        
        % Update vertex u
        function updateNode(obj, s)
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

            obj.U.removeIfPresent(u);

            if s.g ~= s.rhs
                obj.OPEN.insert(s, s.calcKey(obj.currPos));
            end
        end
        
        % return Counter ClockWise neighbor
        function s = ccknbr(obj, s1, s2)
            x = s2.x - s1.x;
            y = s2.y - s1.y;
            
            findComm = [x; y]==obj.moves;
            pos = find(findComm(1, :) & findComm(2, :));
            
            thisPos = pos;
            while true
                thisPos = mod((thisPos-2), 8)+1;
                
                move = obj.moves(:, thisPos);
                if obj.localMap.isInside(s1.x+move(1), s1.y+move(2))
                    if ~obj.localMap.isObstacle(s1.x+move(1), s1.y+move(2))
                        s = obj.localMap.map(s1.x+move(1), s1.y+move(2));
                        return
                    end
                end
                
                if thisPos == pos
                    error("warning!")
                end
            end
        end
        
        % return ClockWise neighbor
        function s = cknbr(obj, s1, s2)
            x = s2.x - s1.x;
            y = s2.y - s1.y;
            
            findComm = [x; y]==obj.moves;
            pos = find(findComm(1, :) & findComm(2, :));
            
            thisPos = pos;
            while true
                thisPos = mod((thisPos), 8)+1;
                
                move = obj.moves(:, thisPos);
                if obj.localMap.isInside(s1.x+move(1), s1.y+move(2))
                    if ~obj.localMap.isObstacle(s1.x+move(1), s1.y+move(2))
                        s = obj.localMap.map(s1.x+move(1), s1.y+move(2));
                        return
                    end
                end
                
                if thisPos == pos
                    error("warning!")
                end
            end
        end
        
        % Compute the shortest path from the goal to the current position
        function computeShortestPath(obj)
            while true
                [s, k] = obj.OPEN.top();
                if ~(min2(k, obj.currPos.calcKey(obj.currPos)) || ...
                    obj.currPos.rhs ~= obj.currPos.g)
                    return
                end
                
                if s.state == State.UNKNOWN || s.state == State.EMPTY || ...
                        s.state == State.VISITED || s.state == State.FUTUREPATH
                    s.state = State.OPEN;
                end

                if (s.g > s.rhs)
                    s.g = s.rhs;
                    obj.OPEN.remove(s);
                    pred = obj.predecessor(s);
                    for s1=pred
%                         if ~obj.OPEN.has(s1)
%                             %s1.g = Inf;
%                             %s1.rhs = Inf;
%                         end
                        
                        s2 = obj.ccknbr(s1, s);
                        if s1.rhs > s1.computeCost(s, s2)
                            s1.rhs = s1.computeCost(s, s2);
                            s1.bptr = s;
                        end
                        s2 = obj.cknbr(s1, s);
                        if s1.rhs > s1.computeCost(s, s2)
                            s1.rhs = s1.computeCost(s, s2);
                            s1.bptr = s2;
                        end
                        
                        obj.updateNode(s1);
                    end
                else
                    s.g = inf;
                    pred = obj.predecessor(s);
                    for s1=pred
                        if s1.bptr == s || s1.bptr == obj.cknbr(s1, s)
                            minV = inf;
                            minP = [];
                            suc = obj.successor(s1);
                            for s2=suc
                                ss = obj.ccknbr(s1, s2);
                                curr = s1.computeCost(s2, ss);
                                if curr < minV
                                    minV = curr;
                                    minP = ss;
                                end
                            end
                            s1.rhs = minV;
                            s1.bptr = minP;
                        end
                        obj.updateNode(s1);
                    end
                    obj.updateNode(s);
                end
                
                if isempty(obj.OPEN.queue)
                    return
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
                [s, ~] = updateCells.pop();
                
                if ~(s == obj.goal)
                    minV = inf;
                    suc = obj.successor(s);
                    for s1=suc
                        ss = obj.ccknbr(s, s1);
                        curr = s.computeCost(s1, ss);
                        if curr < minV
                            minV = curr;
                        end
                    end
                    s.rhs = minV;
                    obj.updateNode(s);
                end
            end
        end
        
        
        % Takes a step from the current position
        function step(obj)
            %move to minPos
            obj.currPos.state = State.PATH;
            [~, obj.currPos] = minVal(obj.currPos, obj.successor(obj.currPos));
            obj.currPos.state = State.POSITION;
            
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


