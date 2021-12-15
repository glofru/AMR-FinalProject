clear all;
clc

%% Main

global dim Sstart Sgoal globalObstacles robotLocVal obstacleVal goalVal unknownVal moves g
global map obstacles newObstacles U g rhs

goalVal = 2;
robotLocVal = 5;
unknownVal = 7;
obstacleVal = 10;

dim = [5, 3];
Sstart = [1; 1];
Sgoal = [5; 3];

globalObstacles = [[2; 2], [3; 2], [4; 2]];

obstacles = [];
newObstacles = [];

moves = [[1; 0], [1; 1], [0; 1], [-1; 1], [-1; 0], [-1; -1], [0; -1], [1; -1]];

Initialize();

inizializeGlobalMap();
inizializeMap();
updateMap();
newObstacles = [];

ComputeShortestPath();

plotMap(map);
disp(g)

while(any(Sstart ~= Sgoal))
    [i, j] = in(Sstart);
    
    if g(i, j) == +inf
        disp("No possible path!");
        return
    end
    
    minV = +inf;
    minPos = [-1; -1];
    succ = u_succ(Sstart);
    for s=succ
        [is, js] = in(s');
        curr = c(Sstart, s') + g(is, js);
        if curr < minV
            minV = curr;
            minPos = [is; js];
        end
    end
    
    %move to minPos
    Sstart = minPos;
    
%     % scan graph
%     isChanged = updateMap();
%     
%     % update graph
%     if isChanged
%         updateEdgesCost();
% %         Initialize();
%         ComputeShortestPath();
%     end
    
    plotMap(map);
    disp(g)
    
end
disp("Goal reached!");


%% Functions

function inizializeGlobalMap()
    % TODO
    global globalMap dim globalObstacles obstacleVal
    globalMap = zeros(dim);
    for o=globalObstacles
        [i, j] = in(o);
        globalMap(i, j) = obstacleVal;
    end
end

function inizializeMap()
    % TODO
    global map dim Sgoal robotLocVal Sstart goalVal unknownVal
    [s1, s2] = in(dim);
    map(1:s1,1:s2) = unknownVal;
    [i, j] = in(Sstart);
    map(i, j) = robotLocVal;
    [i, j] = in(Sgoal);
    map(i, j) = goalVal;
end

function isChanged = updateMap()
    global globalMap map Sstart obstacleVal obstacles newObstacles
    
    [is, js] = in(Sstart);
    isChanged = false;
    
    for i=-1:1
        for j=-1:1
            if isInside([is+i, js+j])
                chr = globalMap(is+i, js+j);
                map(is+i, js+j) = chr;
                
                if chr == obstacleVal
                    new_obs = [is+i, js+j];
                    if ~isAlredyIn(obstacles, new_obs')
                        obstacles(:, end+1) = new_obs';
                        newObstacles(:, end+1) = new_obs';
                        isChanged = true;
                    end
                end
            end
        end
    end

end

function plotMap(map)
    global dim obstacleVal unknownVal Sstart Sgoal
    %myMap = [1 1 1; 1 0 0; 0 1 0; 0 0 1; 0 0 0];
    %heatmap(map, 'Colormap', myMap);
    [s1, s2] = in(dim);
    disp("|-----|");
    matChr(1:s1,1:s2) = "";
    for i=1:s1
        for j=1:s2
            posType = map(i, j);
            if posType == obstacleVal
                chr = "█";
            elseif posType == unknownVal
                chr = "▓";
            else
                chr = "░"; %
            end
            
            matChr(i, j) = chr;
        end
    end
    
    [i, j] = in(Sstart);
    matChr(i, j) = "☺";
    [i, j] = in(Sgoal);
    matChr(i, j) = "☼";
    
    for i=1:s1
        out = "";
        for j=1:s2
            out = out + matChr(i, j);
        end
        disp("|"+out+"|");
    end
    
    disp("|-----|"+newline);
end

function res = h(s1, s2)
    res = norm(s1 - s2);
end

function res = c(s1, s2)
    res = 1;
end

function [i, j] = in(s)
    % return s = [a, b] as 2 different values
    % is equal to do i = s(1); j = s(2); <==> [i, j] = in(s)
    
    i = s(1);
    j = s(2);
end

function res = isInside(s)
    % check if s is inside the map
    
    global dim
    [x, y] = in(s);
    
    if x < 1 || x > dim(1)
        res = false;
        return;
    end
    if y < 1 || y > dim(2)
        res = false;
        return;
    end
    res = true;
end

function isIn = isAlredyIn(L, val)
    % check if val is inside list L
    
    isIn = false;
    for elem=L
        if all(elem==val)
            isIn = true;
            break
        end
    end
end

function Lp = u_pred(u)
    global moves obstacles
    
    Lp = [];
    for m=moves
        pred_pos = u+m;
        
        %se dentro i bordi
        if ~isInside(pred_pos)
            continue
        end
        
        isNotObs = true;
        for o=obstacles
            if all(o'==pred_pos)
                isNotObs = false;
                break
            end
        end
        
        if isNotObs
            % TODO
            if ~isAlredyIn(Lp, pred_pos)
                Lp(:, end+1) = pred_pos;
            end
        end
    end
end

function Ls = u_succ(u)
    global moves obstacles
    
    Ls = [];
    for m=moves
        succ_pos = u+m;
        
        %se dentro i bordi
        if ~isInside(succ_pos)
            continue
        end
        
        isNotObs = true;
        for o=obstacles
            if all(o'==succ_pos)
                isNotObs = false;
                break
            end
        end
        
        if isNotObs
            % TODO
            if ~isAlredyIn(Ls, succ_pos)
                Ls(:, end+1) = succ_pos;
            end
        end
    end
end

function K = calcKey(s)
    [i, j] = in(s);
    
    global g rhs Sstart
    k1 = min(g(i, j), rhs(i, j) + h(Sstart, s));
    k2 = min(g(i, j), rhs(i, j));
    
    K = [k1, k2];
end

function updateVertex(u)
    global Sgoal U g rhs
    
    [i, j] = in(u);
    
    if any(u ~= Sgoal)
        minV = +inf;
        succ = u_succ(u);
        for s=succ
            [is, js] = in(s);
        	curr = c(u, s) + g(is, js);
            if curr < minV
                minV = curr;
            end
        end
        rhs(i, j) = minV;
    end
    
    if U.has(u)
        U = U.remove(u);
    end
    
    if g(i, j) ~= rhs(i, j)
        U = U.update(u, calcKey(u));
    end
end

function Initialize()
    global dim g rhs U Sgoal
    g = Inf(dim);
    rhs = Inf(dim);
    U = PriorityQueue();
    
    [i, j] = in(Sgoal);
    
    rhs(i, j) = 0;
    U = U.insert([i, j], calcKey(Sgoal));
end

function ComputeShortestPath()
    global Sstart g rhs U
    
    [i, j] = in(Sstart);
    
    while (min2(U.topKey(), calcKey(Sstart)) || ...
            rhs(i, j) ~= g(i, j))
        [U, u] = U.pop();
        [ui, uj] = in(u);
        
        if (g(ui, uj) > rhs(ui, uj))
            g(ui, uj) = rhs(ui, uj);
        else
            g(ui, uj) = +inf;
            updateVertex(u);
        end
        
        pred = u_pred(u);
        for k=1:length(pred)
            s = pred(:, k);
            updateVertex(s);
        end
    end
end

function updateEdgesCost()
    global U dim newObstacles
    
    % updato tutti i predecessori degli ostacoli nuovi
    % li metto in una lista e estraggo il più vicino al goal
    
    updateCells = []
    
    
    for o=newObstacles
        pred = u_pred(o');
        
        for p=pred
            if ~isAlredyIn(updateCells, p)
                updateCells(:, end+1) = p;
            end
        end
    end
    
    
    
    %for all directed edges (u, v)
    %    update edge cost c(u, v)
    %    updateVertex(u)
    %end
    
    [s1, s2] = in(dim);
    for i=1:s1
        for j=1:s2
            if belong2U([i, j])
                U{iu, ju} = calcKey(U{iu, ju});
            end
        end
    end
end

