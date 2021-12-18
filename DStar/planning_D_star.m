clear all;
clc
%refresh_path()

%% Main

moves = [[1; 0], [1; 1], [0; 1], [-1; 1], [-1; 0], [-1; -1], [0; -1], [1; -1]];

obsts = [[2; 2], [3; 2], [4; 2]];
m = Map(5, 3, obsts);

start = m.map(1, 1);
start.state = Map.MAP_START;
goal = m.map(5, 3);
goal.state = Map.MAP_GOAL;

disp("Initial Map!")
m.print_map();

d = D_Star(moves, m, goal);

d.run(start, goal);
m.print_map();


% % Fuck MATLAB
% function refresh_path()
%     rehash path
%     savepath
% end