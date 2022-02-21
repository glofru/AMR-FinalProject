classdef D_star_lite_v2_opt < handle
    % Optimized implementation of D* lite v2 from the paper:
    % "Fast Replanning for Navigation in Unknown Terrain"
    
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
        % previous state
        Slast;
        % km parameter
        km;
        
        % if true plot map
        plotVideo;
    end
    
    methods
        % D_star_lite_v2_opt constructor
        function obj = D_star_lite_v2_opt(globalMap, obstacles, Sstart, Sgoal,...
                moves, range, cost, plotVideo)
            % copy vals
            obj.globalMap = globalMap;
            obj.moves = moves;
            obj.U = PriorityQueue();
            obj.km = 0;
            obj.newObstacles = [];
            obj.range = range;
            obj.cost = cost;
            obj.plotVideo = plotVideo;
            
            obj.localMap = Map(obj.globalMap.row, obj.globalMap.col,...
                obstacles, Map.TYPE_UNKNOWN, cost);
            
            obj.currPos = obj.localMap.map(Sstart(1), Sstart(2));
            obj.currPos.state = State.POSITION;
            obj.Slast = obj.currPos;
            obj.goal = obj.localMap.map(Sgoal(1), Sgoal(2));
            obj.goal.state = State.GOAL;
            
            obj.goal.rhs = 0;
            obj.U.insert(obj.goal, [obj.currPos.h(obj.goal), 0]);

            % first scan
            obj.updateMap();
            
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
        
        % return the set of predecessor states of the state u
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
        
        % return the set of successor states of the state u
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
        
        % update vertex u
        function updateVertex(obj, u)
            if u.g ~= u.rhs
                obj.U.insert(u, u.calcKey(obj.currPos, obj.km));
            elseif u.g == u.rhs && obj.U.has(u)
                obj.U.remove(u);
            end
%             
%             if u ~= obj.goal
%                 [u.rhs, ~] = minVal(u, obj.successor(u));
%             end
% 
%             obj.U.removeIfPresent(u);
% 
%             if u.g ~= u.rhs
%                 obj.U.insert(u, u.calcKey(obj.currPos, obj.km));
%             end
        end
        
        % compute the shortest path from the goal to the current position
        function computeShortestPath(obj)
            while true
                [u, Kold] = obj.U.top();
                if ~(min2(Kold, obj.currPos.calcKey(obj.currPos, obj.km)) || ...
                    obj.currPos.rhs ~= obj.currPos.g)
                    return
                end
                Knew = u.calcKey(obj.currPos, obj.km);
                
                if u.state == State.UNKNOWN || u.state == State.EMPTY || ...
                        u.state == State.VISITED || u.state == State.FUTUREPATH
                    u.state = State.OPEN;
                end
                
                if Kold < Knew
                    obj.U.insert(u, Knew);
                elseif (u.g > u.rhs)
                    u.g = u.rhs;
                    obj.U.remove(u);
                    
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
                                [p.rhs, ~] = minVal(p, obj.successor(p));
                            end
                        end
                        obj.updateVertex(p);
                    end
                end
            end
        end

        % update the cost of all the cells needed when new obstacles are
        % discovered
        function updateEdgesCost(obj)
            updateCells = PriorityQueue();
            updateCells.insert(obj.currPos, obj.currPos.calcKey(obj.currPos, obj.km));
            
            for o=obj.newObstacles
                oState = obj.localMap.map(o(1), o(2));
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
                [u, k_old] = updateCells.extract(1);
                Cold = u.c(obj.currPos);
                %obj.updateVertex(u);
                %k = u.calcKey(obj.currPos, obj.km);
                
                if Cold > u.c(obj.currPos)
                    u.rhs = min(u.rhs, u.c(obj.currPos) + obj.currPos.g);
                elseif u.rhs == Cold + obj.currPos.g
                    if u ~= obj.goal
                        [u.rhs, ~] = minVal(u, obj.successor(u));
                    end
                end
                obj.updateVertex(u);
                
                
                
%                 if ~(k == k_old)
%                     pred = obj.predecessor(u);
% 
%                     for p=pred
%                         if ~updateCells.has(p)
%                             updateCells.insert(p, p.calcKey(obj.currPos, obj.km));
%                         end
%                     end
%                 end
            end
        end
        
        
        % compute one step from the current position
        function step(obj)
            %move to minPos
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
        
        % run the algorithm until reach the end
        function run(obj)
            while(~isFinish(obj))
                obj.step()
            end
        end
        
        
        % switch the cells on the shortest path to the goal from oldState
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
        
        % generate the map image
        function rgbImage = buildImageMap(obj)
            obj.switchForFuturePath(State.OPEN, State.FUTUREPATH);
            
            rgbImage = obj.localMap.buildImageMap();
            
            obj.switchForFuturePath(State.FUTUREPATH, State.OPEN);
        end
        
        % plot the map image
        function plot(obj)
            J = obj.buildImageMap();
            imshow(J);
        end
    end
end


