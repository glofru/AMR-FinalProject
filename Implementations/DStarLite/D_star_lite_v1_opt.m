classdef D_star_lite_v1_opt < handle
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
        
        
        plotVideo;
    end
    
    methods
        % D_star_lite_v1 constructor
        function obj = D_star_lite_v1_opt(globalMap, obstacles, Sstart, Sgoal,...
                moves, range, cost, plotVideo)
            % copy vals
            obj.globalMap = globalMap;
            obj.moves = moves;
            obj.U = PriorityQueue();
            obj.newObstacles = [];
            obj.range = range;
            obj.cost = cost;
            obj.plotVideo = plotVideo;
            
            % inizialize map
            obj.localMap = Map(obj.globalMap.row, obj.globalMap.col,...
                obstacles, Map.TYPE_UNKNOWN, cost);
            
            obj.currPos = obj.localMap.map(Sstart(1), Sstart(2));
            obj.goal = obj.localMap.map(Sgoal(1), Sgoal(2));
            
            obj.goal.rhs = 0;
            obj.U.insert(obj.goal, obj.goal.calcKey(obj.currPos));

            % first scan
            obj.updateMap();
            
            % TODO optimize
            % compute first path
            obj.computeShortestPath2();
            
            obj.currPos.state = State.POSITION;
            obj.goal.state = State.GOAL;
        end
        
        % compute the shortest path from the goal to the current position
        function computeShortestPath2(obj)
            while true
                [u, k] = obj.U.top();
                if ~(min2(k, obj.currPos.calcKey(obj.currPos)) || ...
                    obj.currPos.rhs ~= obj.currPos.g)
                    return
                end
                u.state = State.OPEN;
%                 obj.localMap.plot(); % comment for fast plot
%                 pause(0.01)
                
                obj.U.remove(u);
                
                % TODO
                if u.state == State.UNKNOWN || u.state == State.EMPTY || ...
                        u.state == State.VISITED || u.state == State.OPEN
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
        
        
        % check if the algorithm is finished
        function isFin = isFinish(obj)
            if obj.currPos == obj.goal
                disp("goal reach")
                isFin = true;
            elseif obj.currPos.g == inf
                disp("Path not found")
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
                        
                        state = obj.localMap.map(is+i, js+j);
                        if chr == State.OBSTACLE
                            if state.state ~= chr
                                state.state = chr;
%                                 state.g = Inf;
%                                 state.rhs = Inf;
%                                 state.k = state.calcKey(obj.currPos);
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
            if u ~= obj.goal
                [u.rhs, ~] = minVal(u, obj.successor(u));
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
            while true
                [u, k] = obj.U.top();
                if ~(min2(k, obj.currPos.calcKey(obj.currPos)) || ...
                    obj.currPos.rhs ~= obj.currPos.g)
                    return
                end
                u.state = State.OPEN;
                
%                 if obj.plotVideo
%                     obj.localMap.plot();
%                     pause(0.01);
%                 end
                
                obj.U.remove(u);
                
                % TODO
                if u.state == State.UNKNOWN || u.state == State.EMPTY || ...
                        u.state == State.VISITED || u.state == State.OPEN
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
            
             updateCells.insert(obj.currPos, obj.currPos.calcKey(obj.currPos));
            
            for o=obj.newObstacles
                oState = obj.localMap.map(o(1), o(2));
                oState.g = inf;
                oState.rhs = inf;
                oState.k = oState.calcKey(obj.currPos);
%                 
% %                 updateCells.insert(oState, oState.calcKey(obj.currPos))
% %                 pred = obj.predecessor(oState);
% % 
% %                 for p=pred
% %                     updateCells.insert(p, p.calcKey(obj.currPos));
% %                 end
            end
            obj.newObstacles = [];

            %for all directed edges (u, v)
            %    update edge cost c(u, v)
            %    updateVertex(u)
            %end

            while ~updateCells.isEmpty()
                [s, k_old] = updateCells.extract(1);
                obj.updateVertex(s);
%                 if ~(s.k == k_old)
%                     pred = obj.predecessor(s);
% 
%                     for p=pred
%                         updateCells.insert(p, p.calcKey(obj.currPos));
%                     end
%                 end
            end
        end
        
        % return the set of successor states of the state u
        function Ls = successor2(obj, u)
            Ls = State.empty(length(obj.moves), 0);
            currI = 1;
            for m=obj.moves
                x = u.x + m(1);
                y = u.y + m(2);

                if obj.localMap.isInside(x, y)
                    Ls(currI) = obj.localMap.map(x, y);
                    currI = currI+1;
                end
            end
        end
        
        
        % compute one step from the current position
        function step(obj)
%             % move to minPos
%             obj.currPos.state = State.PATH; % TODO
%             [~, nextState] = minVal(obj.currPos, obj.successor2(obj.currPos));
% 
%             % scan graph
%             isChanged = obj.updateMap();
% 
%             obj.localMap.plot();
%             pause(0.01); % because otherwise matlab doesn't update the plot
% 
%             % update graph
%             if nextState.state == State.OBSTACLE %isChanged
%                 % TODO optimize
%                 obj.updateEdgesCost();
%                 obj.computeShortestPath();
%             end
%             
%             obj.currPos.state = State.PATH; % TODO
%             [~, obj.currPos] = minVal(obj.currPos, obj.successor(obj.currPos));
            


            % move to minPos
            obj.currPos.state = State.PATH; % TODO
            [~, obj.currPos] = minVal(obj.currPos, obj.successor(obj.currPos));

            % scan graph
            isChanged = obj.updateMap();
            
            nextStep = obj.currPos;
            while ~isempty(nextStep) && nextStep.state ~= State.FUTUREPATH && nextStep.state ~= State.GOAL
                oldNextStep = nextStep;
                oldNextStep.state = State.FUTUREPATH;
                [~, nextStep] = minVal(oldNextStep, obj.successor(oldNextStep));
            end
            
            if obj.plotVideo
                obj.localMap.plot();
                pause(0.01);
            end
            
            nextStep = obj.currPos;
            while ~isempty(nextStep) && nextStep.state ~= State.VISITED && nextStep.state ~= State.GOAL
                oldNextStep = nextStep;
                oldNextStep.state = State.VISITED;
                [~, nextStep] = minVal(oldNextStep, obj.successor(oldNextStep));
            end
            
            % update graph
            if isChanged
                % TODO optimize
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


