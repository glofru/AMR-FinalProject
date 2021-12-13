clear all;
clc

%% Main
dim = [2, 2];
S = cell(dim)
U = [];

Sstart = [1, 1];
Sgoal = [2, 2];

%Initialize(dim, U, Sgoal);


%% FUNCTIONS

% TODO
    % StateCell h


function [] = Initialize(S, U, Sgoal)
    arguments
        % matrix of all states
        dim
        % priority Queue
        U
        % position of the goal
        Sgoal
    end
    
    U = [];
    %dim = size(S);
    % = [obj(1:length(name)).name]
    
    % TODO ottimizare
    for i=1:dim(1) % row
        for j=1:dim(2) % column
            s = StateCell(+inf, +inf);
            S(i, j) = s;
        end
    end
    S(Sgoal).rhs = 0;
    
    U = PriorityQueue([Sgoal], S(Sgoal).CalcKey());  
end