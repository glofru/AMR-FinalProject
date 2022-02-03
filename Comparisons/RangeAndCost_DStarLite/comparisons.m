clear all
close all
clc
restoredefaultpath

addpath('../utils')
addpath('./DStarLite')

%% LOAD DATA
warning('error', 'MATLAB:deblank:NonStringInput');
inputPath = strcat(uigetdir('', 'Select Input Directory'), '\');

[initParams, infosAlgo] = loadDataFromADAT(inputPath);

%% HEATMAP
heatMap = zeros(initParams.Na, initParams.dim(1), initParams.dim(2));

for e=1:initParams.epochDone
    for n=1:initParams.Na
        for k=1:infosAlgo(e, n).pathLenght
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

waitInput();

%% Comparisons
initTimes4Epoch = zeros(initParams.epochDone, initParams.Na);
computationTimes4Epoch = zeros(initParams.epochDone, initParams.Na);
expCells4Epoch = zeros(initParams.epochDone, initParams.Na);
totSteps4Epoch = zeros(initParams.epochDone, initParams.Na);
pathLenght4Epoch = zeros(initParams.epochDone, initParams.Na);

for i=1:initParams.epochDone
    for j=1:initParams.Na
        initTimes4Epoch(i, j) = infosAlgo(i, j).initTime;
        computationTimes4Epoch(i, j) = infosAlgo(i, j).computationTime;
        expCells4Epoch(i, j) = infosAlgo(i, j).expCells;
        totSteps4Epoch(i, j) = infosAlgo(i, j).totSteps;
        pathLenght4Epoch(i, j) = infosAlgo(i, j).pathLenght;
    end
end

figure
bar(initTimes4Epoch)
title("initTimes4Epoch")

figure
bar(computationTimes4Epoch)
title("computationTimes4Epoch")

figure
bar(expCells4Epoch)
title("expCells4Epoch")

figure
bar(totSteps4Epoch)
title("totSteps4Epoch")

figure
bar(pathLenght4Epoch)
title("pathLenght4Epoch")


figure
subplot(1, 5, 1)
bar(mean(initTimes4Epoch))
title("initTimes4Epoch")
subplot(1, 5, 2)
bar(mean(computationTimes4Epoch))
title("computationTimes4Epoch")
subplot(1, 5, 3)
bar(mean(expCells4Epoch))
title("expCells4Epoch")
subplot(1, 5, 4)
bar(mean(totSteps4Epoch))
title("totSteps4Epoch")
subplot(1, 5, 5)
bar(mean(pathLenght4Epoch))
title("pathLenght4Epoch")


waitInput();

%%

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
boxplot(pathLenght4Epoch)
title("pathLenght4Epoch")

waitInput();

%% FUNCTIONS %%

function waitInput()
    disp("PAUSE: press enter to continue");
    pause();
    disp("CONTINUE!!");
end

