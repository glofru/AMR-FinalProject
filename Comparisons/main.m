clear all
close all
clc
restoredefaultpath

addpath('./utils')

%% LOAD DATA
warning('error', 'MATLAB:deblank:NonStringInput');

disp("Which option?"+newline+...
     "    1) Create a new file for testing"+newline+...
     "    2) Open an old test to be continued"+newline+...
     "    3) Fast test"+newline)
typeOfInput = input('search option: ');
disp(" ");

if ispc
    pathDelimiter = "\";
else
    pathDelimiter = "/";
end

% COLLECTING FILES
switch(typeOfInput)
    case 1
        inputPath = strcat(uigetdir('', 'Select Input Directory'), pathDelimiter);
        inputFile = input("File Name: ", 's')+".adat";
        
        pathAndFile = inputPath+inputFile;
        if isfile(pathAndFile)
            error("Warning: wrong name, file already exist!");
        end
        
        initParams = FileADAT.constByUser();
        
    case 2
        inputPath = strcat(uigetdir('', 'Select Input Directory'), pathDelimiter);
        [inputFile, inputPath] = uigetfile(strcat(inputPath, "\*.adat"), 'MultiSelect', 'off');
        [initParams, ~] = loadDataFromADAT(inputPath, inputFile);
        
    case 3
        initParams = FileADAT.constSTD();
        
    otherwise
        error("Warning: wrong input!");
end

%% MAIN
D1 = initParams.dim(1);
D2 = initParams.dim(2);
epoch = double(input("How many epochs: "));

infosAlgo(epoch, initParams.Na) = AlgoInfo();

% import
switch initParams.typeAlgo
    case FileADAT.ALGO_DS
        addpath('./DStar')
    case FileADAT.ALGO_DSL_V1
        addpath('./DStarLite')
    case FileADAT.ALGO_DSL_V2
        addpath('./DStarLite')
    case FileADAT.ALGO_FDS
        addpath('./FieldDStar')
    otherwise
        error("Wrong input!");
end

for currEpoch=1:epoch
    disp(newline+"<──- Epoch: <strong>"+num2str(currEpoch)+"/"+num2str(epoch)...
        +"</strong> ──->");

    globalObstacles = zeros(2, round(D1*D2/2));
    for i=1:round(D1*D2/2) % 2)
        x = round(mod(rand*D1, D1))+1;
        y = round(mod(rand*D2, D2))+1;

        % obstacles overlap, ok, not an error
        if ~(all([x; y]==initParams.Sstart) || all([x; y]==initParams.Sgoal))
            globalObstacles(:, i) = [x; y];
        end
    end
    
    switch initParams.typeAlgo
        case 1
            map = Map(D1, D2, globalObstacles);
            map.map(initParams.Sstart(1), initParams.Sstart(2)).state = MapState.START;
            map.map(initParams.Sgoal(1), initParams.Sgoal(2)).state = MapState.GOAL;
        otherwise
            map = Map(D1, D2, globalObstacles, Map.TYPE_KNOWN, 1); % TODO cost
            map.map(initParams.Sstart(1), initParams.Sstart(2)).state = State.START;
            map.map(initParams.Sgoal(1), initParams.Sgoal(2)).state = State.GOAL;
    end
    obstacles = [];
    knownObstacles = [];
    
    for i=1:initParams.Na
        disp(newline+"### algorithm[<strong>"+num2str(i)+"</strong>] ###");
        disp("Inizialization");

        switch(initParams.typeAlgo)
            case 1
                tic
                currAlgo = D_Star(map, knownObstacles, initParams.Sstart, initParams.Sgoal,...
                    initParams.moves, initParams.ranges(i), initParams.costs(i));
                tocTime = toc;
            case 2
                tic
                currAlgo = D_star_lite_v1(map, knownObstacles, initParams.Sstart, initParams.Sgoal,...
                    initParams.moves, initParams.ranges(i), initParams.costs(i));
                tocTime = toc;
            case 3
                tic
                currAlgo = D_star_lite_v2(map, knownObstacles, initParams.Sstart, initParams.Sgoal,...
                    initParams.moves, initParams.ranges(i), initParams.costs(i));
                tocTime = toc;
            case 4
                tic
                currAlgo = Field_D_star(map, knownObstacles, initParams.Sstart, initParams.Sgoal,...
                    initParams.moves, initParams.ranges(i), initParams.costs(i));
                tocTime = toc;
            otherwise
                error("Wrong type of algorithm!")
        end

        infosAlgo(currEpoch, i).initTime = tocTime;
        disp("└──-Inizialization terminated in: <strong>"+string(tocTime)+...
            "</strong> s");
        
        disp("Execution");
        tic
        finalPath = currAlgo.run();
        tocTime = toc;
        infosAlgo(currEpoch, i).computationTime = tocTime;
        infosAlgo(currEpoch, i).finalPath = finalPath;
        disp("└──-Execution terminated in: <strong>"+string(tocTime)+...
            "</strong> s");
        
        infosAlgo(currEpoch, i).expCells = currAlgo.expCells;
        infosAlgo(currEpoch, i).expCellsList = currAlgo.expCellsList;
        infosAlgo(currEpoch, i).totSteps = currAlgo.totSteps;
        infosAlgo(currEpoch, i).totStepsList = currAlgo.totStepsList;
        infosAlgo(currEpoch, i).pathLength = currAlgo.pathLength;
    end
end
disp("Terminated!")

initParams.epochDone = epoch;
 switch (typeOfInput)
     case 1
        saveDataOnFileADAT(inputPath, initParams, infosAlgo, inputFile);
     case 2
         saveDataOnFileADAT(inputPath, initParams, infosAlgo, inputFile);
 end

%% FUNCTIONS %%

function waitInput()
    disp("PAUSE: press enter to continue");
    pause();
    disp("CONTINUE!!");
end

function plotAlgs(img_g, algos, Na)
    Nc = ceil(Na/3)+1;
    
    ax1 = subplot(3,Nc,Nc+1);
    image(ax1, img_g);
    title(ax1, "Global Map");
    axis off;
    
    for i=1:Na
        algo = algos(i);
        img_c = algo.localMap.buildImageMap();


        j = floor((i-1)/(Nc-1));
        index = j*Nc+mod((i-1), (Nc-1))+2;

        ax = subplot(3,Nc,index);
        image(ax, img_c);
        title(ax, {"Algorithm Map c="+num2str(algo.cost);...
            "explored cells: "+num2str(algo.getExpCells())+...
            ", tot steps: "+num2str(algo.getTotSteps())+...
            ", path lenght: "+num2str(algo.getPathLength())
            })
        axis off;
        
%         ax = subplot(3,3,3);
%         title(ax, "Algorithm c="+num2str(algo.cost))
%         
%         yyaxis left
%         y_axis1 = diff(algo.expCellsList)';
%         bar(ax, [ y_axis1, zeros(size(y_axis1)) ])
%         ylabel(ax, 'explored cells')
%         
%         yyaxis right
%         y_axis2 = diff(algo.totStepsList)';
%         bar(ax, [ zeros(size(y_axis2)) ,y_axis2])
%         ylabel(ax, "work for step")
    end

    pause(0.25)
end

function WIP_plotAlgs(img_g, algorithm_c1, algorithm_c2, algorithm_c3)
    img_c1 = algorithm_c1.localMap.buildImageMap();
    img_c2 = algorithm_c2.localMap.buildImageMap();
    img_c3 = algorithm_c3.localMap.buildImageMap();
    

    % ### 1 ###
    ax2 = subplot(3,2,1);
    image(ax2, img_c1);
    title(ax2, {"Algorithm Map c=0.1";...
        "explored cells: "+num2str(algorithm_c1.getExpCells())+...
        ", tot steps: "+num2str(algorithm_c1.getTotSteps())+...
        ", path lenght: "+num2str(algorithm_c1.getPathLength())
        })
    axis off;
    
    ax2_2 = subplot(3,2,2);
    title(ax2_2, 'Algorithm c=0.1')
    yyaxis left
    ylabel(ax2_2, 'explored cells')
    y_axis1 = diff(algorithm_c1.expCellsList)';
    bar(ax2_2, [ y_axis1, zeros(size(y_axis1)) ])
    yyaxis right
    ylabel(ax2_2, 'work for step')
    y_axis2 = diff(algorithm_c1.totStepsList)';
    bar(ax2_2, [ zeros(size(y_axis2)) ,y_axis2])

    
    % ### 2 ###
    ax3 = subplot(3,2,3);
    image(ax3, img_c2);
    title(ax3, {"Algorithm Map c=1";...
        "explored cells: "+num2str(algorithm_c2.getExpCells())+...
        ", tot steps: "+num2str(algorithm_c2.getTotSteps())+...
        ", path lenght: "+num2str(algorithm_c2.getPathLength())
        })
    axis off;
    
    ax3_2 = subplot(3,2,4);
    title(ax3_2, 'Algorithm c=1')
    yyaxis left
    ylabel(ax3_2, 'explored cells')
    y_axis1 = diff(algorithm_c2.expCellsList)';
    bar(ax3_2, [ y_axis1, zeros(size(y_axis1)) ])
    yyaxis right
    ylabel(ax3_2, 'work for step')
    y_axis2 = diff(algorithm_c2.totStepsList)';
    bar(ax3_2, [ zeros(size(y_axis2)) ,y_axis2])

    
    % ### 3 ###
    ax4 = subplot(3,2,5);
    image(ax4, img_c3);
    title(ax4, {"Algorithm Map c=2";...
        "explored cells: "+num2str(algorithm_c3.getExpCells())+...
        ", tot steps: "+num2str(algorithm_c3.getTotSteps())+...
        ", path lenght: "+num2str(algorithm_c3.getPathLength())
        })
    axis off;
    
    ax4_2 = subplot(3,2,6);
    title(ax4_2, 'Algorithm c=2')
    yyaxis left
    ylabel(ax4_2, 'explored cells')
    y_axis1 = diff(algorithm_c3.expCellsList)';
    bar(ax4_2, [ y_axis1, zeros(size(y_axis1)) ])
    yyaxis right
    ylabel(ax4_2, 'work for step')
    y_axis2 = diff(algorithm_c3.totStepsList)';
    bar(ax4_2, [ zeros(size(y_axis2)) ,y_axis2])
    
    
    y_axis1 = diff(algorithm_c1.totStepsList)';
    y_axis2 = diff(algorithm_c2.totStepsList)';
    y_axis3 = diff(algorithm_c3.totStepsList)';
    maxDim = max([length(y_axis1), length(y_axis2), length(y_axis3)]);
    y_addon1 = zeros(maxDim-length(y_axis1), 1);
    y_addon2 = zeros(maxDim-length(y_axis2), 1);
    y_addon3 = zeros(maxDim-length(y_axis3), 1);
    
    bar([[y_axis1; y_addon1], [y_axis2; y_addon2], [y_axis3; y_addon3]])
    
    
    
    y_axis1 = diff(algorithm_c1.expCellsList)';
    y_axis2 = diff(algorithm_c2.expCellsList)';
    y_axis3 = diff(algorithm_c3.expCellsList)';
    maxDim = max([length(y_axis1), length(y_axis2), length(y_axis3)]);
    y_addon1 = zeros(maxDim-length(y_axis1), 1);
    y_addon2 = zeros(maxDim-length(y_axis2), 1);
    y_addon3 = zeros(maxDim-length(y_axis3), 1);
    
    bar([[y_axis1; y_addon1], [y_axis2; y_addon2], [y_axis3; y_addon3]])
    
    pause(0.5)
end


