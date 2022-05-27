classdef D_Star < handle
    % Implementation of D* from the paper:
    % "Optimal and Efficient Path Planning for Partially-Known Environments"
    
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
        % List of open states
        open_list;

        % max length of the final path
        maxLengthFinalPath;
        
        % number of explored cell added to the priority queue
        expCells; % for each insert +1
        % number of explored cell added to the priority queue for each step
        expCellsList;
        % number of explored cell during computeShortestPath
        totSteps; % for each cell popped from computeShortestPath +1
        % number of explored cell during computeShortestPath for each step
        totStepsList;
        % number of steps
        pathLength; % for each step +1
        % replanning time
        replanningTime = 0;
        % number of replanning occurencies
        replanningOccurencies = 0;

        continuousPathLength = 0;
    end

    methods
        % D_Star constructor
        function obj = D_Star(globalMap, obstacles, Sstart, Sgoal,...
                moves, range, cost)
            obj.globalMap = globalMap;
            obj.moves = moves;
            obj.range = range;
            obj.cost = cost;


            % initialize map
            obj.localMap = DMap(obj.globalMap.row, obj.globalMap.col, obstacles);
            
            obj.currPos = obj.localMap.map(Sstart(1), Sstart(2));
            obj.currPos.state = MapState.START;
            obj.goal = obj.localMap.map(Sgoal(1), Sgoal(2));
            obj.goal.state = MapState.GOAL;
            
            obj.open_list = OpenList(obj.goal);

            % performance metrics
            obj.expCells = 1; % 1 because the goal is initially put on the open list
            obj.expCellsList = [];
            obj.totSteps = 0;
            obj.totStepsList = [];
            obj.pathLength = 0;
            obj.maxLengthFinalPath = (obj.globalMap.row+obj.globalMap.col)*4;

            % first scan and path computation
            obj.updateMap();
            obj.computeShortestPath();

            obj.expCellsList(end+1) = obj.expCells;
            obj.totStepsList(end+1) = obj.totSteps;
        end

        % States are processed according to the D* paper
        function process_state(obj)
            X = obj.open_list.min_state();
            if isempty(X)
                error("Path not found") % TODO
            end
            Kold = X.k;
            obj.open_list.remove(X);
            
            %obj.localMap.plot(X); % comment for fast plot
            
            succ = obj.localMap.neighbors(X, obj.moves);
            if Kold < X.h
                for Y=succ
                    c = Y.h + Y.cost(X) * obj.cost;
                    if Y.h <= Kold && X.h > c
                        X.parent = Y;
                        X.h = c;
                    end
                end
            end

            if Kold == X.h
                for Y=succ
                    c = X.h + X.cost(Y) * obj.cost;
                    if Y.tag == StateTag.NEW || ...
                            (~isempty(Y.parent) && Y.parent == X && Y.h ~= c) || ...
                            (~isempty(Y.parent) && Y.parent ~= X && Y.h > c)
                        Y.parent = X;
                        obj.expCells = obj.expCells + 1;
                        obj.open_list.insert(Y, c);
                    end
                end
            else
                for Y=succ
                    c = X.h + X.cost(Y) * obj.cost;
                    if Y.tag == StateTag.NEW ||...
                            (Y.parent == X && Y.h ~= c)
                        Y.parent = X;
                        obj.expCells = obj.expCells + 1;
                        obj.open_list.insert(Y, c);
                    else
                        if Y.parent ~= X && Y.h > c
                            obj.expCells = obj.expCells + 1;
                            obj.open_list.insert(Y, X.h);
                        else
                            c = Y.h + Y.cost(X) * obj.cost;
                            if Y.parent ~= X && X.h > c && ...
                                    Y.tag == StateTag.CLOSED && ...
                                    Y.h > Kold
                                obj.expCells = obj.expCells + 1;
                                obj.open_list.insert(Y, Y.h);
                            end
                        end
                    end
                end
            end
        end

        % Cost is changed and the algorithm is run again
        function modify_cost(obj, state)
            if state.tag == StateTag.CLOSED
                obj.open_list.insert(state, state.cost(state.parent))
            end

            while ~obj.open_list.isEmpty()
                obj.process_state();
                if obj.open_list.get_kmin() >= state.h
                    break
                end
            end
        end

        % Check if the algorithm is finished
        function f = isFinish(obj)
            f = obj.currPos == obj.goal;
        end

        % Takes a step from the current position
        function state = step(obj)
            state = 1;

            obj.updateMap()
            
            obj.currPos.state = MapState.PATH;

            % obj.localMap.plot();
            % pause(0.1); % because otherwise matlab doesn't update the plot
            
            if isempty(obj.currPos.parent) || obj.pathLength > obj.maxLengthFinalPath
                error("Path not found")
            end

            % update graph
            if obj.currPos.parent.state == MapState.OBSTACLE
                tic;
                obj.modify_cost(obj.currPos)
                
                obj.replanningTime = obj.replanningTime + toc;
                obj.replanningOccurencies = obj.replanningOccurencies + 1;

                state = 0;
                return
            end

            % metrics
            obj.pathLength = obj.pathLength + 1;
            obj.totStepsList(end+1) = obj.totSteps;
            obj.expCellsList(end+1) = obj.expCells;

            obj.continuousPathLength = obj.continuousPathLength + obj.currPos.g;

            % goes forward
            obj.currPos = obj.currPos.parent;

            obj.continuousPathLength = obj.continuousPathLength - obj.currPos.g;
            
            obj.currPos.state = MapState.PATH;
%             obj.plot();
            pause(0.01);
        end

        % Run the algorithm until it reaches the end
        function [finalPath, averageReplanningTime] = run(obj)
            finalPath = zeros(obj.maxLengthFinalPath, 2);
            dimensionPath = 1;
            finalPath(dimensionPath, :) = [obj.currPos.x, obj.currPos.y];
            
            while(~isFinish(obj))
                state = obj.step();

                if state == 1
                    dimensionPath = dimensionPath + 1;
                    finalPath(dimensionPath, :) = [obj.currPos.x, obj.currPos.y];
                end
            end

            finalPath = finalPath(1:dimensionPath, :);
            averageReplanningTime = obj.replanningTime / obj.replanningOccurencies;
        end

        % Compute the shortest path from the goal to the current position
        function computeShortestPath(obj)
            while obj.currPos.tag ~= StateTag.CLOSED && ~obj.open_list.isEmpty()
                obj.totSteps = obj.totSteps + 1;
                obj.process_state();
            end
        end

        % Scan the map for new obstacles
        function updateMap(obj)
            is = obj.currPos.x;
            js = obj.currPos.y;
            
            r = obj.range;

            for i=-r:r
                for j=-r:r
                    newX = is+i;
                    newY = js+j;
                    
                    if obj.localMap.isInside(newX, newY)
                        s = obj.globalMap.map(newX, newY).state;
                        if s == MapState.OBSTACLE
                            state = obj.localMap.map(newX, newY);
                            if state.state ~= s
                                state.h = Inf;
                                state.k = Inf;
                                state.state = s;
                                state.parent = DState.empty;
                            end
                        end
                    end
                end
            end
        end
        
        
        
        % Plot the map image
        function plot(obj)
            J = obj.localMap.buildImageMap(obj.currPos);
            imshow(J);
        end
        
        
    end
end