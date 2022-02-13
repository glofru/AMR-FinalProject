clear all
close all
clc
restoredefaultpath

addpath('./utils')
%% Main

disp("Which search algorithm?"+newline+...
     "    1) D*"+newline+...
     "    2) D*Lite v1"+newline+...
     "    3) D*Lite v1 optimized"+newline+...
     "    4) D*Lite v2"+newline+...
     "    5) D*Lite v2 optimized"+newline+...
     "    6) Field D*"+newline+...
     "    7) Field D* optimized"+newline)
algorithmType = 3;%input('search algorithm: ');

D1 = 50;
D2 = 50;
numObs = round(D1*D2*0.5);
dim = [D1; D2];
Sstart = [1; 1];
Sgoal = [D1; D2];

range = 2;
cost = 1;

moves = [[1; 0], [1; 1], [0; 1], [-1; 1], [-1; 0], [-1; -1], [0; -1], [1; -1]];

execute = true;
while execute

    globalObstacles = zeros(2, numObs);
    for i=1:numObs
        x = round(mod(rand*(D1-3), D1))+2; % round(mod(rand*D1, D1))+1;
        y = round(mod(rand*(D2-3), D1))+2; % round(mod(rand*D2, D2))+1;

        % obstacles overlap, ok, not an error
        if ~(all([x; y]==Sstart) || all([x; y]==Sgoal))
            globalObstacles(:, i) = [x; y];
        end
    end
    
    switch algorithmType
        case 1
            addpath('./DStar')
        case 2
            addpath('./DStarLite')
        case 3
            addpath('./DStarLite')
        case 4
            addpath('./DStarLite')
        case 5
            addpath('./DStarLite')
        case 6
            addpath('./FieldDStar')
        case 7
            addpath('./FieldDStar')
        otherwise
            error("Wrong input!");
    end

    if algorithmType == 1
        map = Map(dim(1), dim(2), globalObstacles);
        map.map(Sstart(1), Sstart(2)).state = MapState.START;
        map.map(Sgoal(1), Sgoal(2)).state = MapState.GOAL;
    else
        map = Map(dim(1), dim(2), globalObstacles, Map.TYPE_KNOWN, cost);
        map.map(Sstart(1), Sstart(2)).state = State.START;
        map.map(Sgoal(1), Sgoal(2)).state = State.GOAL;
    end
    obstacles = [];
    
    switch algorithmType
        case 1
            tic
            algorithm = D_Star(map, obstacles, Sstart, Sgoal, moves,...
                range, cost);
            tocTime = toc;
        case 2
            tic
            algorithm = D_star_lite_v1(map, obstacles, Sstart, Sgoal, moves,...
                range, cost);
            tocTime = toc;
        case 3
            tic
            algorithm = D_star_lite_v1_opt(map, obstacles, Sstart, Sgoal, moves,...
                range, cost);
            tocTime = toc;
        case 4
            tic
            algorithm = D_star_lite_v2(map, obstacles, Sstart, Sgoal, moves,...
                range, cost);
            tocTime = toc;
        case 5
            tic
            algorithm = D_star_lite_v2_opt(map, obstacles, Sstart, Sgoal, moves,...
                range, cost);
            tocTime = toc;
        case 6
            tic
            algorithm = Field_D_star(map, obstacles, Sstart, Sgoal, moves,...
                range, cost);
            tocTime = toc;
        case 7
            tic
            algorithm = Field_D_star_opt(map, obstacles, Sstart, Sgoal, moves,...
                range, cost);
            tocTime = toc;
    end
    
    disp('Inizialization terminated in: '+string(tocTime)+' s'+newline);
    disp("Global Map and Algorithm Initial Map!");
    
    set(gcf, 'Position', [400 200 1000 400]);
    ax1 = subplot(1, 2, 1);
    map.plot();
    title(ax1, "Global Map")
    ax2 = subplot(1, 2, 2);
    algorithm.localMap.plot();
    title(ax2, "Algorithm Initial Map")
    xlabel(['',newline,'\bf Press Enter to continue!'])
    waitInput();

    tic
    algorithm.run();
    disp('run terminated in: '+string(toc)+' s'+newline);
    
    execute = input("Another map? [0=No/1=Yes] ");
    try
        if execute ~= 0 && execute ~= 1
            execute = 0;
        end
    catch
        execute = 0;
    end
    
end
disp("Terminated!")

%% FUNCTIONS %%

function waitInput()
    disp("PAUSE: press enter to continue");
    pause();
    disp("CONTINUE!!");
end
