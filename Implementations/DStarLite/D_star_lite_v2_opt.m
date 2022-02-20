classdef D_star_lite_v2_opt < handle
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
        % D_star_lite_v2_opt constructor
        function obj = D_star_lite_v2_opt(globalMap, obstacles, Sstart, Sgoal,...
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
                
                plotVideo = 0;
            end

            obj.globalMap = globalMap;
            obj.moves = moves;
            obj.U = PriorityQueue();
            obj.km = 0;
            obj.obstacles = obstacles;
            obj.newObstacles = [];
            obj.range = range;
            obj.cost = cost;
            obj.plotVideo = plotVideo;
            
            % Map initialization
            obj.localMap = Map(obj.globalMap.row, obj.globalMap.col,...
                obj.obstacles, Map.TYPE_UNKNOWN, cost);
            
            obj.currPos = obj.localMap.map(Sstart(1), Sstart(2));
            obj.currPos.state = State.POSITION;
            obj.Slast = obj.currPos;
            obj.goal = obj.localMap.map(Sgoal(1), Sgoal(2));
            obj.goal.state = State.GOAL;
            
            obj.goal.rhs = 0;
            
            %*********************OPTIMIZATION********************
            obj.U.insert(obj.goal, [obj.currPos.h(obj.goal), 0]);

            % First scan and path computation
            obj.updateMap();
            obj.computeShortestPath();
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
                            obj.localMap.map(is+i, js+j).state = chr;
                            
                            new_obs = [is+i, js+j];
                            obj.localMap.map(is+i, js+j).state = State.OBSTACLE;
                            if ~isAlredyIn(obj.obstacles, new_obs')
                                obj.obstacles(:, end+1) = new_obs';
                                obj.newObstacles(:, end+1) = new_obs';
                                isChanged = true;
                            end
                        end
                    end
                end
            end
            obj.currPos.state = State.POSITION;
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
                    % TODO ottimizzare
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
                    % TODO ottimizzare
                    if ~isAlredyIn(Ls, obj_pos)
                        Ls(currI) = obj_pos;
                        currI = currI+1;
                    end
                end
            end
        end
        
        %*******************OPTIMIZATION*******************
        % Update vertex u
        function updateVertex(obj, u)
%             if u.g ~= u.rhs
%                 obj.U.insert(u, u.calcKey(obj.currPos, obj.km));
%             elseif u.g == u.rhs && obj.U.has(u)
%                 obj.U.remove(u);
%             end
            
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
        %*******************FINISH OPTIMIZATION*************
        
        %*******************OPTIMIZATION*******************
        % Compute the shortest path from the goal to the current position
        function computeShortestPath(obj)
            while (min2(obj.U.topKey(), obj.currPos.calcKey(obj.currPos, obj.km)) || ...
                    obj.currPos.rhs ~= obj.currPos.g)
                
%                 if obj.plotVideo
%                     obj.localMap.plot();
%                     pause(0.01);
%                 end
                
                [u, Kold] = obj.U.top();
                Knew = u.calcKey(obj.currPos, obj.km);
                
                % TODO
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
        %*******************FINISH OPTIMIZATION*************
        
        % Update the cost of all the cells needed when new obstacles are
        % discovered
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
        
        % Takes a step from the current position
        function step(obj)

            %move to minPos
            obj.currPos.state = State.PATH; % TODO
            [~, obj.currPos] = minVal(obj.currPos, obj.successor(obj.currPos));
            obj.currPos.state = State.POSITION; % TODO
            
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


