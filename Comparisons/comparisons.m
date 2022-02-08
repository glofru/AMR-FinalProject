clear all
close all
clc
restoredefaultpath

addpath('./utils')

%% LOAD DATA
warning('error', 'MATLAB:deblank:NonStringInput');
if ispc
    pathDelimiter = "\";
else
    pathDelimiter = "/";
end
inputPath = strcat(uigetdir('', 'Select Input Directory'), pathDelimiter);

[initParams, infosAlgo] = loadDataFromADAT(inputPath);

%% HEATMAP
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

waitInput();

%% Comparisons
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
bar(pathLength4Epoch)
title("pathLength4Epoch")


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
bar(mean(pathLength4Epoch))
title("pathLength4Epoch")


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
boxplot(pathLength4Epoch)
title("pathLength4Epoch")

waitInput();

%% FUNCTIONS %%

function waitInput()
    disp("PAUSE: press enter to continue");
    pause();
    disp("CONTINUE!!");
end


