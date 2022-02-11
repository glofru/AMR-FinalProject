clear all
close all
clc
restoredefaultpath

addpath('./utils')
addpath('./DStar')
addpath('./DStarLite')
addpath('./FieldDStar')

%% LOAD DATA
warning('error', 'MATLAB:deblank:NonStringInput');

disp("Which option?"+newline+...
     "    1) Create a new file for testing"+newline+...
     "    2) Open an old test to be continued"+newline+...
     "    3) Fast test"+newline)
typeOfInput = 3;%input('search option: ');
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
numObs = round(D1*D2*4/10);
epoch = double(input("How many epochs: "));

initParams.Na = 4;

infosAlgo(epoch, initParams.Na) = AlgoInfo();
imgs = zeros(D1, D2, 3, initParams.Na);

for currEpoch=1:epoch
    disp(newline+"<──- Epoch: <strong>"+num2str(currEpoch)+"/"+num2str(epoch)...
        +"</strong> ──->");

    globalObstacles = zeros(2, numObs);
    for i=1:numObs
        x = round(mod(rand*D1, D1))+1;
        y = round(mod(rand*D2, D2))+1;

        % obstacles overlap, ok, not an error
        if ~(all([x; y]==initParams.Sstart) || all([x; y]==initParams.Sgoal))
            globalObstacles(:, i) = [x; y];
        end
    end
    
    map1 = DMap(D1, D2, globalObstacles);
    map1.map(initParams.Sstart(1), initParams.Sstart(2)).state = MapState.START;
    map1.map(initParams.Sgoal(1), initParams.Sgoal(2)).state = MapState.GOAL;

    map2 = DSLMap(D1, D2, globalObstacles, DSLMap.TYPE_KNOWN, 1); % TODO cost
    map2.map(initParams.Sstart(1), initParams.Sstart(2)).state = DSLState.START;
    map2.map(initParams.Sgoal(1), initParams.Sgoal(2)).state = DSLState.GOAL;
    
    map3 = FDMap(D1, D2, globalObstacles, DSLMap.TYPE_KNOWN, 1); % TODO cost
    map3.map(initParams.Sstart(1), initParams.Sstart(2)).state = DSLState.START;
    map3.map(initParams.Sgoal(1), initParams.Sgoal(2)).state = DSLState.GOAL;
    
    obstacles = [];
    knownObstacles = [];
    
    for i=1:initParams.Na
        disp(newline+"### algorithm[<strong>"+num2str(i)+"</strong>] ###");
%         disp("Inizialization");

        switch(i)
            case 1
                tic
                currAlgo = D_Star(map1, knownObstacles, initParams.Sstart, initParams.Sgoal,...
                    initParams.moves, initParams.ranges(i), initParams.costs(i));
                tocTime = toc;
            case 2
                tic
                currAlgo = D_star_lite_v1(map2, knownObstacles, initParams.Sstart, initParams.Sgoal,...
                    initParams.moves, initParams.ranges(i), initParams.costs(i));
                tocTime = toc;
            case 3
                tic
                currAlgo = D_star_lite_v2(map2, knownObstacles, initParams.Sstart, initParams.Sgoal,...
                    initParams.moves, initParams.ranges(i), initParams.costs(i));
                tocTime = toc;
            case 4
                tic
                currAlgo = Field_D_star(map3, knownObstacles, initParams.Sstart, initParams.Sgoal,...
                    initParams.moves, initParams.ranges(i), initParams.costs(i));
                tocTime = toc;
            otherwise
                error("Wrong type of algorithm!")
        end

        infosAlgo(currEpoch, i).initTime = tocTime;
%         disp("└──-Inizialization terminated in: <strong>"+string(tocTime)+...
%             "</strong> s");
%         
%         disp("Execution");
        tic
        finalPath = currAlgo.run();
        tocTime = toc;
        infosAlgo(currEpoch, i).computationTime = tocTime;
        infosAlgo(currEpoch, i).finalPath = finalPath;
%         disp("└──-Execution terminated in: <strong>"+string(tocTime)+...
%             "</strong> s");
        
        infosAlgo(currEpoch, i).expCells = currAlgo.expCells;
        infosAlgo(currEpoch, i).expCellsList = currAlgo.expCellsList;
        infosAlgo(currEpoch, i).totSteps = currAlgo.totSteps;
        infosAlgo(currEpoch, i).totStepsList = currAlgo.totStepsList;
        infosAlgo(currEpoch, i).pathLength = currAlgo.pathLength;
        
        imgs(:, :, :, i) = currAlgo.localMap.buildImageMap();
    end
    
    %plotAlgs(map2.buildImageMap(), imgs, initParams.Na)
end
disp("Terminated!")

initParams.epochDone = epoch;
 switch (typeOfInput)
     case 1
        saveDataOnFileADAT(inputPath, initParams, infosAlgo, inputFile);
     case 2
         saveDataOnFileADAT(inputPath, initParams, infosAlgo, inputFile);
 end
 
%%

heatMap = zeros(initParams.Na, initParams.dim(1), initParams.dim(2));
for e=1:initParams.epochDone
    for n=1:initParams.Na
        for k=1:infosAlgo(e, n).pathLength
            pos = infosAlgo(e, n).finalPath(k, :);
            heatMap(n, pos(1), pos(2)) = heatMap(n, pos(1), pos(2)) + 1;
        end
    end
end

figure
num = ceil(sqrt(initParams.Na));

for i=1:initParams.Na
    subplot(num, num, i)
    colormap('hot')
    imagesc(reshape(heatMap(i, :, :), [initParams.dim(1), initParams.dim(2)]))
    colorbar
    title("Algorithm r= "+num2str(initParams.ranges(i))+...
        "c="+num2str(initParams.costs(i)))
end


initTimes4Epoch = zeros(initParams.epochDone, initParams.Na);
computationTimes4Epoch = zeros(initParams.epochDone, initParams.Na);
expCells4Epoch = zeros(initParams.epochDone, initParams.Na);
totSteps4Epoch = zeros(initParams.epochDone, initParams.Na);
pathLength4Epoch = zeros(initParams.epochDone, initParams.Na);

for i=1:initParams.epochDone
    for j=1:initParams.Na
        initTimes4Epoch(i, j) = infosAlgo(i, j).initTime;
        computationTimes4Epoch(i, j) = infosAlgo(i, j).computationTime;
        expCells4Epoch(i, j) = infosAlgo(i, j).expCells;
        totSteps4Epoch(i, j) = infosAlgo(i, j).totSteps;
        pathLength4Epoch(i, j) = infosAlgo(i, j).pathLength;
    end
end
 
figure
subplot(1, 5, 1)
boxplot(initTimes4Epoch)
title("initTimes4Epoch")
subplot(1, 5, 2)
boxplot(computationTimes4Epoch)
title("computationTimes4Epoch")
subplot(1, 5, 3)
boxplot(expCells4Epoch)
title("expCells4Epoch")
subplot(1, 5, 4)
boxplot(totSteps4Epoch)
title("totSteps4Epoch")
subplot(1, 5, 5)
boxplot(pathLength4Epoch)
title("pathLength4Epoch")

%% FUNCTIONS %%

function waitInput()
    disp("PAUSE: press enter to continue");
    pause();
    disp("CONTINUE!!");
end

function plotAlgs(img_g, imgs, Na)
    Nc = ceil(Na/3)+1;
    
    ax1 = subplot(3,Nc,Nc+1);
    image(ax1, img_g);
    title(ax1, "Global Map");
    axis off;
    
    for i=1:Na
        %algo = algos(i);
        img_c = imgs(:, :, :, i);% algo.localMap.buildImageMap();


        j = floor((i-1)/(Nc-1));
        index = j*Nc+mod((i-1), (Nc-1))+2;

        ax = subplot(3,Nc,index);
        image(ax, img_c);
%         title(ax, {"Algorithm Map c="+num2str(algo.cost);...
%             "explored cells: "+num2str(algo.getExpCells())+...
%             ", tot steps: "+num2str(algo.getTotSteps())+...
%             ", path lenght: "+num2str(algo.getPathLength())
%             })
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
