clear all;
clc

%% Main

disp("Whitc search algorithm?"+newline+...
     "    1) D*"+newline+...
     "    2) D*Lite v1"+newline+...
     "    3) D*Lite v2"+newline)
algorithm = input('search algorithm: ');

dim = [5, 5];
Sstart = [1; 1];
Sgoal = [5; 5];

globalObstacles = [[2; 2], [3; 2], [4; 2], [5; 2],...
                   [2; 4], [3; 4], [4; 4]];
moves = [[1; 0], [1; 1], [0; 1], [-1; 1], [-1; 0], [-1; -1], [0; -1], [1; -1]];

switch algorithm
    case 1
        addpath('./DStar')
        map = Map(dim(1), dim(2), globalObstacles);

        Sstart = map.map(Sstart(1), Sstart(2));
        Sstart.state = Map.MAP_START;
        Sgoal = map.map(Sgoal(1), Sgoal(2));
        Sgoal.state = Map.MAP_GOAL;

        disp("Initial Map!")
        map.print_map();

        algorithm = D_Star(moves, map, Sgoal);
        algorithm.run(Sstart, Sgoal);
        
    case 2
        addpath('./DStarLite')
        map = Map(dim(1), dim(2), globalObstacles, Map.TYPE_KNOWN);
        map.map(Sstart(1), Sstart(2)).state = Map.MAP_START;
        map.map(Sgoal(1), Sgoal(2)).state = Map.MAP_GOAL;

        disp("Global Map!")
        map.plotMap();

        obstacles = [];
        algorithm = D_star_lite_v1(map, obstacles, Sstart, Sgoal, moves);
        disp("Initial Map!")
        algorithm.localMap.plotMap();
        algorithm.run();
        
    case 3
        
        
    otherwise
        error("Wrong!");
end


