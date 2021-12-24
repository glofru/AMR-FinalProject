clear all;
clc

%% Main

disp("Whitc search algorithm?"+newline+...
     "    1) D*"+newline+...
     "    2) D*Lite v1"+newline+...
     "    3) D*Lite v2"+newline)
algorithmType = input('search algorithm: ');

L = 5;
dim = [L, L];
Sstart = [1; 1];
Sgoal = [L; L];

execute = true;
while execute

    globalObstacles = zeros(2, round(L*L/2));
    for i=1:round(L*L/2)
        x = round(mod(rand*L, L))+1;
        y = round(mod(rand*L, L))+1;

        % obstacles overlap, ok, not an error
        if all([x; y]==Sstart) || all([x; y]==Sgoal)
            [x; y]
            i = i-1;
        else
            globalObstacles(:, i) = [x; y];
        end
    end

    moves = [[1; 0], [1; 1], [0; 1], [-1; 1], [-1; 0], [-1; -1], [0; -1], [1; -1]];

    switch algorithmType
        case 1
            tic
            addpath('./DStar')
            map = Map(dim(1), dim(2), globalObstacles);

            SstartMap = map.map(Sstart(1), Sstart(2));
            SstartMap.state = Map.MAP_START;
            SgoalMap = map.map(Sgoal(1), Sgoal(2));
            SgoalMap.state = Map.MAP_GOAL;

            disp("Initial Map!")
            map.print_map();

            algorithm = D_Star(moves, map, SgoalMap);
            disp('Inizialization terminated in: '+string(toc)+' s'+newline);
            pause()

            tic
            algorithm.run(SstartMap, SgoalMap);
            disp('run terminated in: '+string(toc)+' s'+newline);

        case 2
            tic
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
            disp('Inizialization terminated in: '+string(toc)+' s'+newline);
            pause()

            tic
            algorithm.run();
            disp('run terminated in: '+string(toc)+' s'+newline);

        case 3


        otherwise
            error("Wrong!");
    end
    
    execute = input("Another map? [0=No/1=Yes] ");
    if execute ~= 0 && execute ~= 1
        execute = 0;
    end
    
end
disp("Terminated!")


