clear all
close all
clc
restoredefaultpath

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


