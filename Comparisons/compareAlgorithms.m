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
inputPath = strcat(uigetdir('', 'Select Input Directory'), '\');
[inputFiles, inputPath] = uigetfile(strcat(inputPath, "/*.adat"), 'MultiSelect', 'on');

n = size(inputFiles, 2);
if ~iscellstr(inputFiles) || n <= 1
    disp("You have to select at least two files")
    return
end

initParams = {};
infosAlgo = {};
algos = {};
for i=1:n
    [initParams{i}, infosAlgo{i}] = loadDataFromADAT(inputPath, inputFiles{i});
    algos{i} = initParams{i}.typeAlgoToStr();
end

epochs = initParams{1}.epochDone;
Na = initParams{1}.Na;

%% Comparisons
initTimes4Epoch = zeros(epochs, n);
computationTimes4Epoch = zeros(epochs, n);
expCells4Epoch = zeros(epochs, n);
totSteps4Epoch = zeros(epochs, n);
pathLength4Epoch = zeros(epochs, n);


for i=1:epochs
    for j=1:n
        % compute average values
        initTime = 0;
        computationTime = 0;
        expCells = 0;
        totSteps = 0;
        pathLength = 0;
        for k=1:Na
            initTime = initTime + infosAlgo{j}(i, k).initTime;
            computationTime = computationTime + infosAlgo{j}(i, k).computationTime;
            expCells = expCells + infosAlgo{j}(i, k).expCells;
            totSteps = totSteps + infosAlgo{j}(i, k).totSteps;
            pathLength = pathLength + infosAlgo{j}(i, k).pathLength;
        end
        initTime = initTime / Na;
        computationTime = computationTime / Na;
        expCells = expCells / Na;
        totSteps = totSteps / Na;
        pathLength = pathLength / Na;

        initTimes4Epoch(i, j) = initTime;
        computationTimes4Epoch(i, j) = computationTime;
        expCells4Epoch(i, j) = expCells;
        totSteps4Epoch(i, j) = totSteps;
        pathLength4Epoch(i, j) = pathLength;
    end
end

figure
bar(initTimes4Epoch)
title("Initialization times")
xlabel("Epoch")
ylabel("Time (s)")
grid on;
legend(algos);

figure
bar(computationTimes4Epoch)
title("Running time")
xlabel("Epoch")
ylabel("Time (s)")
grid on;
legend(algos);

figure
bar(expCells4Epoch)
title("Explored cells")
xlabel("Epoch")
ylabel("Number of cells")
grid on;
legend(algos);

figure
bar(totSteps4Epoch)
title("Total algorithm steps")
xlabel("Epoch")
ylabel("Number of steps")
grid on;
legend(algos);

figure
bar(pathLength4Epoch)
title("Path length")
xlabel("Epoch")
ylabel("Path length")
grid on;
legend(algos);


figure
subplot(1, 5, 1)
bar(mean(initTimes4Epoch))
title("Initialization time")
grid on;
xlabel("Algorithm")
ylabel("Time (s)")

subplot(1, 5, 2)
bar(mean(computationTimes4Epoch))
title("Running time")
xlabel("Algorithm")
ylabel("Time (s)")
grid on;

subplot(1, 5, 3)
bar(mean(expCells4Epoch))
title("Explored cells")
xlabel("Algorithm")
ylabel("Number of cells")
grid on;

subplot(1, 5, 4)
bar(mean(totSteps4Epoch))
title("Total algorithm steps")
xlabel("Algorithm")
ylabel("Number of steps")
grid on;

subplot(1, 5, 5)
bar(mean(pathLength4Epoch))
title("Path Length")
xlabel("Algorithm")
ylabel("Path length")
grid on;

waitInput();

%%

figure
subplot(1, 5, 1)
boxplot(initTimes4Epoch)
title("Initialization time")
grid on;
xlabel("Algorithm")
ylabel("Time (s)")

subplot(1, 5, 2)
boxplot(computationTimes4Epoch)
title("Running time")
xlabel("Algorithm")
ylabel("Time (s)")
grid on;

subplot(1, 5, 3)
boxplot(expCells4Epoch)
title("Explored cells")
xlabel("Algorithm")
ylabel("Number of cells")
grid on;

subplot(1, 5, 4)
boxplot(totSteps4Epoch)
title("Total algorithm steps")
xlabel("Algorithm")
ylabel("Number of steps")
grid on;

subplot(1, 5, 5)
boxplot(pathLength4Epoch)
title("Path length")
xlabel("Algorithm")
ylabel("Path length")
grid on;

waitInput();

%% FUNCTIONS %%

function waitInput()
    disp("PAUSE: press enter to continue");
    pause();
    disp("CONTINUE!!");
end