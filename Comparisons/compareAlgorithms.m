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
[inputFiles, inputPath] = uigetfile(strcat(pwd, pathDelimiter, "*.adat"), 'MultiSelect', 'on');

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
    algos{i} = initParams{i}.typeAlgoToStr(initParams{i}.typeAlgo);
end

epochs = initParams{1}.epochDone;
Na = initParams{1}.Na;

%% Dot graph
colors = ["r.", "b.", "g."];
initTimeDot = {};
% computationTimeDot = zeros(n, epochs*Na);
% expCellsDot = zeros(n, epochs*Na);
% totStepsDot = zeros(n, epochs*Na);
% pathLengthDot = zeros(n, epochs*Na);

for i=1:Na
    initTimeDot{i} = zeros(n, epochs);
    for j=1:n
        for k=1:epochs
            initTimeDot{i}(j, k) = infosAlgo{j}(k, i).initTime;
        end
    end
end

figure
for i=1:Na
    subplot(1, Na, i)
    for j=1:n
        plot(ones(1, epochs), initTimeDot{i}(j, :), colors(j))
        hold on
    end
    grid on
    legend(algos)
end

waitInput()

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

%%
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
colors = [
    222/255, 104/255, 95/255;
    87/255, 201/255, 161/255;
    90/255, 87/255, 201/255;
];

figure
subplot(1, 5, 1)
boxplot(initTimes4Epoch)
% add colors
h = findobj(gca,'Tag','Box');
for j=1:length(h)
    patch(get(h(j),'XData'),get(h(j),'YData'),colors(j,:),'FaceAlpha',.5);
end
title("Initialization time")
grid on;
xlabel("Algorithm")
ylabel("Time (s)")
legend(flip(algos))

subplot(1, 5, 2)
boxplot(computationTimes4Epoch)
% add colors
h = findobj(gca,'Tag','Box');
for j=1:length(h)
    patch(get(h(j),'XData'),get(h(j),'YData'),colors(j,:),'FaceAlpha',.5);
end
title("Running time")
xlabel("Algorithm")
ylabel("Time (s)")
grid on;
legend(flip(algos))

subplot(1, 5, 3)
boxplot(expCells4Epoch)
% add colors
h = findobj(gca,'Tag','Box');
for j=1:length(h)
    patch(get(h(j),'XData'),get(h(j),'YData'),colors(j,:),'FaceAlpha',.5);
end
title("Explored cells")
xlabel("Algorithm")
ylabel("Number of cells")
grid on;
legend(flip(algos))

subplot(1, 5, 4)
boxplot(totSteps4Epoch)
% add colors
h = findobj(gca,'Tag','Box');
for j=1:length(h)
    patch(get(h(j),'XData'),get(h(j),'YData'),colors(j,:),'FaceAlpha',.5);
end
title("Total algorithm steps")
xlabel("Algorithm")
ylabel("Number of steps")
grid on;
legend(flip(algos))

subplot(1, 5, 5)
boxplot(pathLength4Epoch)
% add colors
h = findobj(gca,'Tag','Box');
for j=1:length(h)
    patch(get(h(j),'XData'),get(h(j),'YData'),colors(j,:),'FaceAlpha',.5);
end
title("Path length")
xlabel("Algorithm")
ylabel("Path length")
grid on;
legend(flip(algos))

waitInput();

%% FUNCTIONS %%

function waitInput()
    disp("PAUSE: press enter to continue");
    pause();
    disp("CONTINUE!!");
end