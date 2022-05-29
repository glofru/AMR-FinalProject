clear all
close all
clc
restoredefaultpath

addpath('./utils')

%% Main

if ispc
    pathDelimiter = "\";
else
    pathDelimiter = "/";
end

disp("Which search algorithm?"+newline+...
     "    1) D*"+newline+...
     "    2) D*Lite v1"+newline+...
     "    3) D*Lite v1 optimized"+newline+...
     "    4) D*Lite v2"+newline+...
     "    5) D*Lite v2 optimized"+newline+...
     "    6) Field D*"+newline+...
     "    7) Field D* optimized"+newline)
% algorithmType = input('search algorithm: ');
algorithmType = 1;

switch algorithmType
    case 1
        addpath('./DStar')
        nameAlgo = "DStar";
    case 2
        addpath('./DStarLite')
        nameAlgo = "DStarLiteV1";
    case 3
        addpath('./DStarLite')
        nameAlgo = "DStarLiteV1OPT";
    case 4
        addpath('./DStarLite')
        nameAlgo = "DStarLiteV2";
    case 5
        addpath('./DStarLite')
        nameAlgo = "DStarLiteV2OPT";
    case 6
        addpath('./FieldDStar')
        nameAlgo = "FieldDStar";
    case 7
        addpath('./FieldDStar')
        nameAlgo = "FieldDStarOPT";
    otherwise
        error("Wrong input!");
end

% plotVideo = input("Plot video? [0=No/1=Yes] ");
% saveVideo = input("Save video? [0=No/1=Yes] ");
plotVideo = 0;
saveVideo = 0;

range = 3;
cost = 1.5;

moves = [[1; 0], [1; 1], [0; 1], [-1; 1], [-1; 0], [-1; -1], [0; -1], [1; -1]];

%% EXECUTION

execute = true;
while execute
    
%     genRandom = input("Generate map at random? [0=No/1=Yes] ");
    genRandom = 1;
    
    if genRandom
        D1 = 20;
        D2 = 20;
        numObs = round(D1*D2*0.25);
        dim = [D1; D2];
        Sstart = [1; 1];
        Sgoal = [D1; D2];
        
        globalObstacles = zeros(2, numObs);
        for i=1:numObs
            x = round(mod(rand*(D1-3), D1))+2;
            y = round(mod(rand*(D2-3), D1))+2;

            % obstacles overlap, ok, not an error
            if ~(all([x; y]==Sstart) || all([x; y]==Sgoal))
                globalObstacles(:, i) = [x; y];
            end
        end

    else
        mMap = ['░░░░░░░░░░░░░░░░░░░░';
                '░████████░░░░░░░░░░░';
                '░█░░░░░░█░░░░░░░░░░░';
                '░█░░░░░░█░░░░░░░░░░░';
                '░█░░░░░░░░░░░░░░░░░░';
                '░█░░░░░░█░░░░░░░░░░░';
                '░█░░░░░░█░░░░░░░░░░░';
                '░███░████░░░░░██████';
                '░░░░░░░░░░░░░░█░░░░░';
                '░░░░░░░░░░░░░░░░░░░░';
                '░░░░░░░░░░░░░░░░░░░░';
                '░░░░░░░░░░░░░░█░░░░░';
                '░░░░░░░░░░░░░░██████';
                '░░███░░███░░░░░░░░░░';
                '░░█░░░░░░█░░░░░░░░░░';
                '░░█░░░░░░█░░░░░░░░░░';
                '░░█░░░░░░█░░░░░░░░░░';
                '░░█░░░░░░█░░░░░░░░░░';
                '░░███░░███░░░░░░░░░░';
                '░░░░░░░░░░░░░░░░░░░░'];

        globalObstacles = [];
        [s1, s2] = size(mMap);
        for i=1:s1
            for j=1:s2
                if mMap(i, j) == '█'
                    globalObstacles(:, end+1) = [i; j];
                end
            end
        end
        
        D1 = s1;
        D2 = s2;
        numObs = round(D1*D2*0.25);
        dim = [D1; D2];
        Sstart = [1; 1];
        Sgoal = [D1; D2];
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

    % create pgm file to export the map into CoppeliaSim
    convert_obstacle_to_pgm(map, "map.pgm")

    obstacles = [];

    writerObj_p = 0;
    if saveVideo
        d = dir(strcat(pwd, pathDelimiter, nameAlgo, "_*.avi"));
        
        fileNameVideo = strcat(nameAlgo, "_", num2str(length(d)));
        disp("Saving video on file: "+fileNameVideo)
        writerObj = VideoWriter(fileNameVideo);
        writerObj.FrameRate = 30;
        open(writerObj);
        
        fileNameVideo = strcat(nameAlgo, "_", num2str(length(d)+1));
        disp("Saving video on file: "+fileNameVideo)
        writerObj_p = VideoWriter(fileNameVideo);
        writerObj_p.FrameRate = 30;
        open(writerObj_p);
    end
    
    switch algorithmType
        case 1
            tic
            algorithm = D_Star(map, obstacles, Sstart, Sgoal, moves,...
                range, cost, plotVideo, saveVideo, writerObj_p);
            tocTime = toc;
        case 2
            tic
            algorithm = D_star_lite_v1(map, obstacles, Sstart, Sgoal, moves,...
                range, cost, plotVideo, saveVideo, writerObj_p);
            tocTime = toc;
        case 3
            tic
            algorithm = D_star_lite_v1_opt(map, obstacles, Sstart, Sgoal, moves,...
                range, cost, plotVideo);
            tocTime = toc;
        case 4
            tic
            algorithm = D_star_lite_v2(map, obstacles, Sstart, Sgoal, moves,...
                range, cost, plotVideo, saveVideo, writerObj_p);
            tocTime = toc;
        case 5
            tic
            algorithm = D_star_lite_v2_opt(map, obstacles, Sstart, Sgoal, moves,...
                range, cost, plotVideo);
            tocTime = toc;
        case 6
            tic
            algorithm = Field_D_star(map, obstacles, Sstart, Sgoal, moves,...
                range, cost, plotVideo, saveVideo, writerObj_p);
            tocTime = toc;
        case 7
            tic
            algorithm = Field_D_star_opt(map, obstacles, Sstart, Sgoal, moves,...
                range, cost, plotVideo);
            tocTime = toc;
    end
    if saveVideo
        close(writerObj_p);
    end
    
    disp('Inizialization terminated in: '+string(tocTime)+' s'+newline);
    disp("Global Map and Algorithm Initial Map!");
    
    if plotVideo
        set(gcf, 'Position', [400 200 1000 400]);
        ax1 = subplot(1, 2, 1);
        map.plot();
        title(ax1, "Global Map")
        ax2 = subplot(1, 2, 2);
        algorithm.localMap.plot();
        title(ax2, "Algorithm Initial Map")
        xlabel(['',newline,'\bf Press Enter to continue!'])
        plotLegend();
        waitInput();
    end
    
    %return
    if saveVideo
        tic
        while(~algorithm.isFinish())
            algorithm.step();
            
            frame = algorithm.buildImageMap();
            writeVideo(writerObj, frame);
            writeVideo(writerObj, frame);
            writeVideo(writerObj, frame);
        end
        disp('run terminated in: '+string(toc)+' s'+newline);
        close(writerObj);
    else
        tic
        path = algorithm.run();

        disp('run terminated in: '+string(toc)+' s'+newline);
    end
    
    if plotVideo
        plotLegend();
    end

    coppeliaSimulation = true;
    if coppeliaSimulation
        %interpolation (linear, makima, spline, etc)
        size_path = size(path);
        interpolation_dt = 0.1; %it will generate 10 new points every 1 original point
        xq = 0:interpolation_dt:size_path(1);
        path_x = fliplr(interp1(path(:,1),xq,'makima'));
        path_y = fliplr(interp1(path(:,2),xq,'makima'));
        path_theta = fliplr(interp1(path(:,3),xq,'makima'));
        path_v = fliplr(interp1(path(:,5),xq,'makima'));
        path_w = fliplr(interp1(path(:,6),xq,'makima'));
        path = [path_x' path_y' path_theta' path_theta' path_v' path_w'];
        size_path = size(path);

        image = imread('map.pgm');
        state_robot = [Sstart(1) + 0.1, Sstart(2) - 0.1, 0.8, 0, 0, 0];
        scale = 1;
        dt = 0.05;
        [rgbImage,real_robot] = input_output_linearization(image, state_robot, path, scale, Sgoal, dt);

        %plotting
        figure(); plot(path(:,1),path(:,2)); hold on; plot(real_robot(:,1),real_robot(:,2))
        figure(); J = imresize(rgbImage, 5); imshow(J);
    end
    
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

function plotLegend()
    colorArr = [[0, 0, 0]; [0.75, 0.75, 0.75];...
                [0, 0, 1]; [1, 0, 0]; [1, 1, 0];...
                [0.2, 0.8, 0.2]; [0.5, 0, 0.5]; [1, 0, 0]; [0.2, 0.6, 1]];
    labelsArr = ["OBSTACLE"; "UNKNOWN";...
        "START"; "GOAL"; "POSITION";...
        "VISITED"; "OPEN"; "PATH"; "FUTUREPATH"];
    hold on;
    for ii = 1:length(labelsArr)
      scatter([],[],1, colorArr(ii, :), 'filled', 'DisplayName', labelsArr{ii});
    end
    hold off;
    legend();
end


