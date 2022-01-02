classdef Field_D_star < handle
    properties
        globalMap;
        
        localMap;
        currPos;
        goal;
        moves;
        
        U;
        obstacles;
        newObstacles;
        
        
        
        map_limit;
        map;
        start;
        cells_isopen;
        size_x;
        size_y;
        resolution;
        maxIter;
    end
    
    methods
        function obj = Field_D_star(initial_state,sampling_time,limit,goal,map,resolution,maxIter)
            obj.map_limit = limit; %limit/resolution
            obj.goal = int16(goal/resolution);
            obj.start = [int16(initial_state(1)/resolution) int16(initial_state(2)/resolution)];
            obj.resolution = resolution;
            obj.maxIter = maxIter;
            %obj.map = im2double(map);
            %[x, y, g, h, open, parent id (the position in the array)]
            obj.map = zeros(size(map,1)*size(map,2),6);
            obj.size_x = size(map,1);
            obj.size_y = size(map,2);
            %[open, cost]
%             obj.cells_isopen = zeros(size(map,1)*size(map,2),3);
            %obj.cells_closed = zeros(size(map,1)*size(map,2),1);
            
            for i = 1:size(map,1)
               for j = 1:size(map,2) 
                  if(map(i,j) < 250) 
%                       obj.cells_isopen(i +(j-1)*size(map,2),:) = [-1,10000,10000];
                      obj.obstacles = [obj.obstacles, [i; j]];
%                   else
%                       %h = abs(obj.goal(1)-((i-1)*resolution)) + abs(obj.goal(2)-((j-1)*resolution));
%                       h = 0;
%                       obj.map(i +(j-1)*size(map,2),:) = [i,j,0,h,0,0];
%                       if(i == obj.start(1) && j == obj.start(2))
%                         obj.cells_isopen(i + (j-1)*size(map,2),:) = [1,0,0];
%                       else
%                         obj.cells_isopen(i + (j-1)*size(map,2),:) = [0,10000,10000];
%                       end
                  end
               end
            end
            
            D1 = obj.size_x;
            D2 = obj.size_y;
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
            obj.moves = [[1; 0], [1; 1], [0; 1], [-1; 1], [-1; 0], [-1; -1], [0; -1], [1; -1]];
            obj.U = PriorityQueue();
            %obj.obstacles = obstacles;
            obj.newObstacles = [];
            
            % inizialize map
            obj.localMap = Map(obj.size_x, obj.size_y, obj.obstacles, Map.TYPE_UNKNOWN);
            
            obj.currPos = obj.localMap.map(obj.start(1), obj.start(2));
            obj.currPos.state = MapState.POSITION;
            obj.goal = obj.localMap.map(obj.goal(1), obj.goal(2));
            obj.goal.state = MapState.GOAL;
            
            % inizialize state vals
            for i=1:obj.localMap.row
                for j=1:obj.localMap.col
                    obj.localMap.map(i, j).g = inf;
                    obj.localMap.map(i, j).rhs = inf;
                end
            end
            
            obj.goal.rhs = 0;
            obj.U = obj.U.insert(obj.goal, obj.goal.calcKey(obj.currPos));

            % first scan
            obj.updateMap();
            
            %tic
            % TODO optimize
            % compute first path
            obj.computeShortestPath();
            %disp('computeShortestPath: '+string(toc)+' s'+newline);
        end
        
        function isIn = isAlredyIn(obj, L, val) % TODO
            % check if val is inside list L

            isIn = false;
            for elem=L
                if all(elem==val)
                    isIn = true;
                    break
                end
            end
        end

        function isChanged = updateMap(obj)
            isChanged = false;
            
            is = obj.currPos.x;
            js = obj.currPos.y;

            for i=-1:1
                for j=-1:1
                    if obj.localMap.isInside(is+i, js+j)
                        chr = obj.globalMap(is+i, js+j);
                        
                        % TODO
                        %obj.localMap.map(is+i, js+j).state = chr;
                            
                        if chr < 250% == MapState.OBSTACLE
                            new_obs = [is+i, js+j];
                            obj.localMap.map(is+i, js+j).state = MapState.OBSTACLE;
                            if ~obj.isAlredyIn(obj.obstacles, new_obs')
                                obj.obstacles(:, end+1) = new_obs';
                                obj.newObstacles(:, end+1) = new_obs';
                                isChanged = true;
                            end
                        end
                    end
                end
            end
            obj.currPos.state = MapState.POSITION;
        end
        
        function Lp = predecessor(obj, u)
            Lp = State.empty(1, 0);
            for m=obj.moves
                pred_pos = [u.x; u.y]+m;

                %se dentro i bordi
                if ~obj.localMap.isInside(pred_pos(1), pred_pos(2))
                    continue
                end

                isNotObs = true;
                for o=obj.obstacles
                    if all(o==pred_pos)
                        isNotObs = false;
                        break
                    end
                end

                if isNotObs
                    % TODO ottimizzare
                    pred_pos = obj.localMap.map(pred_pos(1), pred_pos(2));
                    if ~obj.isAlredyIn(Lp, pred_pos)
                        Lp(end+1) = pred_pos;
                    end
                end
            end
        end
        
        function Ls = sucessor(obj, u)
            Ls = State.empty(1, 0);
            for m=obj.moves
                pred_pos = [u.x; u.y]+m;

                %se dentro i bordi
                if ~obj.localMap.isInside(pred_pos(1), pred_pos(2))
                    continue
                end

                isNotObs = true;
                for o=obj.obstacles
                    if all(o==pred_pos)
                        isNotObs = false;
                        break
                    end
                end

                if isNotObs
                    % TODO ottimizzare
                    pred_pos = obj.localMap.map(pred_pos(1), pred_pos(2));
                    if ~obj.isAlredyIn(Ls, pred_pos)
                        Ls(end+1) = pred_pos;
                    end
                end
            end
        end
        
         % s1, s2 are neighbourds
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
            
            c = 1;
            b = 0.5;
            
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
            
            % return vs;
        end
        
        function updateVertex(obj, u)
            if u ~= obj.goal
                minV = inf;
                 connbrs = obj.sucessor(u);
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
                obj.U = obj.U.remove(u);
            end

            if u.g ~= u.rhs
                obj.U = obj.U.insert(u, u.calcKey(obj.currPos));
            end
        end
        
        function computeShortestPath(obj)
            if obj.U.isEmpty()
                    return
            end
                
            while (min2(obj.U.topKey(), obj.currPos.calcKey(obj.currPos)) || ...
                    obj.currPos.rhs ~= obj.currPos.g)
                obj.localMap.plotMap();
                [obj.U, u] = obj.U.pop();
                
                % TODO
                if u.state == MapState.UNKNOWN || u.state == MapState.EMPTY || ...
                        u.state == MapState.VISITED
                    u.state = MapState.START;
                end

                if (u.g > u.rhs)
                    u.g = u.rhs;
                else
                    u.g = inf;
                    obj.updateVertex(u);
                end

                pred = obj.predecessor(u);
                for p=pred
                    obj.updateVertex(p);
                end

                if obj.U.isEmpty()
                    return
                end
            end
        end

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
                        updateCells = updateCells.insert(p, p.calcKey(obj.currPos));
                    end
                end
            end
            obj.newObstacles = [];


            %for all directed edges (u, v)
            %    update edge cost c(u, v)
            %    updateVertex(u)
            %end

            while ~updateCells.isEmpty()
                [updateCells, s, k_old] = updateCells.pop();
                obj.updateVertex(s);
                k = s.calcKey(obj.currPos);
                if ~(k == k_old)
                    pred = obj.predecessor(s);

                    for p=pred
                        if ~updateCells.has(p)
                            updateCells = updateCells.insert(p, p.calcKey(obj.currPos));
                        end
                    end
                end
            end
        end
        
        function final_path = run(obj)
            final_path = ones(obj.maxIter,6);
            dimension_path = 1;
            final_path(dimension_path,1:2) = [obj.currPos.x, obj.currPos.y]; 
            
            while(obj.currPos ~= obj.goal && dimension_path < obj.maxIter)
                if obj.currPos.g == inf
                    disp("No possible path!");
                    return
                end

                minV = inf;
                minPos = State.empty(1, 0);
                succ = obj.sucessor(obj.currPos);
                for s=succ
                    curr = obj.currPos.c(s) + s.g;
                    if curr < minV
                        minV = curr;
                        minPos = s;
                    end
                end

                %move to minPos
                obj.currPos.state = MapState.PATH; % TODO
                obj.currPos = minPos;
                
                dimension_path = dimension_path + 1;
                final_path(dimension_path,1:2) = [obj.currPos.x, obj.currPos.y]; 

                % scan graph
                isChanged = obj.updateMap();
                
                obj.localMap.plotMap();

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