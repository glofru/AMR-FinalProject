clear all;
clc

%% Main

global dim Sstart Sgoal obstacles robotLocVal obstacleVal goalVal moves g

goalVal = 2;
robotLocVal = 5;
obstacleVal = 10;

dim = [5, 3];
Sstart = [1, 1];
Sgoal = [5, 3];

obstacles = [[2, 2]; [3, 2]; [4, 2]]';
moves = [[1, 0]; [1, 1]; [0, 1]; [-1, 1]; [-1, 0]; [-1, -1]; [0, -1]; [1, -1]]';

Initialize();
updateMap();

ComputeShortestPath();
[i, j] = in(Sstart);

while(any(Sstart ~= Sgoal))
    if g(i, j) == +inf
        disp("No possible path!");
        return
    end
    
    minV = +inf;
    minPos = [-1, -1];
    succ = u_succ(Sstart);
    for s=succ
        [is, js] = in(s');
        curr = c(Sstart, s') + g(is, js);
        if curr < minV
            minV = curr;
            minPos = [is, js];
        end
    end
    
    %move to minPos
    Sstart = minPos;
    
    updateMap();
    plotMap();
    
    % scan graph
    
end
disp("Goal reached!");


%% Functions

function updateMap()
    % TODO
    global map dim Sgoal obstacles robotLocVal obstacleVal Sstart goalVal
    map = zeros(dim);
    for o=obstacles
        [i, j] = in(o);
        map(i, j) = obstacleVal;
    end
    [i, j] = in(Sstart);
    map(i, j) = robotLocVal;
    [i, j] = in(Sgoal);
    map(i, j) = goalVal;
end

function plotMap()
    global map dim robotLocVal obstacleVal goalVal
    %myMap = [1 1 1; 1 0 0; 0 1 0; 0 0 1; 0 0 0];
    %heatmap(map, 'Colormap', myMap);
    [s1, s2] = in(dim);
    disp("|-----|");
    for i=1:s1
        out = "";
        for j=1:s2
            posType = map(i, j);
            if posType == robotLocVal
                chr = "☺";
            elseif posType == obstacleVal
                chr = "█";
            elseif posType == goalVal
                chr = "☼";
            else
                chr = "░";
            end
            out = out + chr;
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
    i = s(1);
    j = s(2);
end

function res = min2(v1, v2)
    if (v1(1) < v2(1))
        res = true;
    elseif (v1(1) == v2(1))
        if (v1(2) < v2(2))
            res = true;
        elseif (v1(2) == v2(2))
            res = false;
        else % (v1(2) > v2(2))
            res = false;
        end
    else % (v1(1) > v2(1))
        res = false;
    end
end

function minV = U_topKey()
    global U
    
    minV = [];
    [s1, s2] = size(U);
    for i=1:s1
        for j=1:s2
            if ~isempty(U{i, j})
                if isempty(minV) || min2(U{i, j}, minV)
                    minV = U{i, j};
                end
            end
        end
    end
end

function minU = U_top()
    global U
    
    minV = [];
    minU = [];
    [s1, s2] = size(U);
    for i=1:s1
        for j=1:s2
            if ~isempty(U{i, j})
                if isempty(minV) || min2(U{i, j}, minV)
                    minV = U{i, j};
                    minU = [i, j];
                end
            end
        end
    end
end

function U_remove(u)
    global U

    [i, j] = in(u);
    U(i, j) = {double.empty(0)};
end

function u = U_pop()
    u = U_top();
    U_remove(u);
end

function Lp = u_pred(u)
    global moves dim obstacles
    
    Lp = [];
    for m=moves
        pred_pos = u+m';
        
        %se dentro i bordi
        [x, y] = in(pred_pos);
        if x < 1 || x > dim(1)
            continue
        end
        if y < 1 || y > dim(2)
            continue
        end
        
        isNotObs = true;
        for o=obstacles
            if all(o'==u)
                isNotObs = false;
                break
            end
        end
        
        if isNotObs
            % TODO
            isNotIn = true;
            for val=Lp
                if all(val==pred_pos)
                    isNotIn = false;
                    break
                end
            end
            if isNotIn
                Lp(:, end+1) = pred_pos';
            end
        end
    end
end

function Ls = u_succ(u)
    global moves dim obstacles
    
    Ls = [];
    for m=moves
        succ_pos = u+m';
        
        %se dentro i bordi
        [x, y] = in(succ_pos);
        if x < 1 || x > dim(1)
            continue
        end
        if y < 1 || y > dim(2)
            continue
        end
        
        isNotObs = true;
        for o=obstacles
            if all(o'==u)
                isNotObs = false;
                break
            end
        end
        
        if isNotObs
            % TODO
            isNotIn = true;
            for val=Ls
                if all(val==succ_pos)
                    isNotIn = false;
                    break
                end
            end
            if isNotIn
                Ls(:, end+1) = succ_pos';
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

function res = belong2U(u)
    global U

    [i, j] = in(u);
    res = ~isempty(cell2mat(U(i, j)));
end

function updateVertex(u)
    global Sgoal U g rhs
    
    [i, j] = in(u);
    
    if any(u ~= Sgoal)
        minV = +inf;
        succ = u_succ(u);
        for s=succ
            [is, js] = in(s');
        	curr = c(u, s') + g(is, js);
            if curr < minV
                minV = curr;
            end
        end
        rhs(i, j) = minV;
    end
    
    if belong2U(u)
        U_remove(u);
    end
    
    if g(i, j) ~= rhs(i, j)
        U{i, j} = calcKey(u);
    end
end

function [] = Initialize()
    global dim g rhs U Sgoal
    g = Inf(dim);
    rhs = Inf(dim);
    U = cell(dim);
    
    [i, j] = in(Sgoal);
    
    rhs(i, j) = 0;
    U{i, j} = calcKey(Sgoal);
end

function [] = ComputeShortestPath()
    global Sstart g rhs
    
    [i, j] = in(Sstart);
    
    while (min2(U_topKey(), calcKey(Sstart)) || ...
            rhs(i, j) ~= g(i, j))
        u = U_pop();
        [ui, uj] = in(u);
        
        if (g(ui, uj) > rhs(ui, uj))
            g(ui, uj) = rhs(ui, uj);
            pred = u_pred(u);
            for i=1:length(pred)
                s = pred(:, i)';
                updateVertex(s);
            end
        else
            g(ui, uj) = +inf;
            updateVertex(u);
            %for s in pred(u)
            %    updateVertex(s);
            %end
        end
    end
end


