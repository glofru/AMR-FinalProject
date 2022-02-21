classdef D_Star < handle
    % Implementation of D* from the paper:
    % "Optimal and Efficient Path Planning for Partially-Known Environments"
    
    properties
        globalMap;

        localMap;
        currPos;
        goal;
        moves;

        range;
        cost;

        open_list;
        
        
        plotVideo;
    end

    methods
        function obj = D_Star(globalMap, obstacles, Sstart, Sgoal,...
                moves, range, cost, plotVideo)
            obj.globalMap = globalMap;
            obj.moves = moves;
            obj.range = range;
            obj.cost = cost;
            obj.plotVideo = plotVideo;


            % initialize map
            obj.localMap = Map(obj.globalMap.row, obj.globalMap.col, obstacles);
            
            obj.currPos = obj.localMap.map(Sstart(1), Sstart(2));
            obj.currPos.state = MapState.START;
            obj.goal = obj.localMap.map(Sgoal(1), Sgoal(2));
            obj.goal.state = MapState.GOAL;
            
            obj.open_list = OpenList(obj.goal);

            % first scan and path computation
            obj.updateMap();
            obj.computeShortestPath();
        end
        
        
        function process_state(obj)
            X = obj.open_list.min_state();
            if isempty(X)
                error("Path not found") % TODO
            end
            Kold = X.k;
            obj.open_list.remove(X);
% 
%             if obj.plotVideo
%                 obj.localMap.plot();
%                 pause(0.01);
%             end
            
            succ = obj.localMap.neighbors(X, obj.moves);
            if Kold < X.h
                for Y=succ
                    if Y.h <= Kold && X.h > Y.h + Y.cost(X)
                        X.parent = Y;
                        X.h = Y.h + Y.cost(X);
                    end
                end
            end

            if Kold == X.h
                for Y=succ
                    if Y.tag == StateTag.NEW || ...
                            (~isempty(Y.parent) && Y.parent == X && Y.h ~= X.h + X.cost(Y)) || ...
                            (~isempty(Y.parent) && Y.parent ~= X && Y.h > X.h + X.cost(Y))
                        Y.parent = X;
                        obj.open_list.insert(Y, X.h + X.cost(Y));
                    end
                end
            else
                for Y=succ
                    if Y.tag == StateTag.NEW ||...
                            (Y.parent == X && Y.h ~= X.h + X.cost(Y))
                        Y.parent = X;
                        obj.open_list.insert(Y, X.h + X.cost(Y));
                    else
                        if Y.parent ~= X && Y.h > X.h + X.cost(Y)
                            obj.open_list.insert(Y, X.h);
                        else
                            if Y.parent ~= X && X.h > Y.h + Y.cost(X) && ...
                                    Y.tag == StateTag.CLOSED && ...
                                    Y.h > Kold
                                obj.open_list.insert(Y, Y.h);
                            end
                        end
                    end
                end
            end
        end

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

        function f = isFinish(obj)
            f = obj.currPos == obj.goal;
        end

        function step(obj)
            obj.updateMap()

            obj.currPos.state = MapState.PATH;
            
            % update graph
            
            if isempty(obj.currPos.parent)
                disp("Path not found")
                return
            end
            
            if obj.currPos.parent.state == MapState.OBSTACLE
                obj.modify_cost(obj.currPos)
                return
            end

            % goes forward
            obj.currPos = obj.currPos.parent;
            obj.currPos.state = MapState.CURPOS;
            
            if obj.plotVideo
                obj.plot();
                pause(0.01);
            end
        end

        function run(obj)
            while(~isFinish(obj))
                obj.step()
            end
        end

        function computeShortestPath(obj)
            while obj.currPos.tag ~= StateTag.CLOSED && ~obj.open_list.isEmpty()
                obj.process_state();
            end
        end

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
                                state.parent = State.empty;
                            end
                        end
                    end
                end
            end
        end
        
        function rgbImage = buildImageMap(obj)
            rgbImage = obj.localMap.buildImageMap(obj.currPos);
        end
        
        function plot(obj)
            J = obj.buildImageMap();
            imshow(J);
        end
    end
end