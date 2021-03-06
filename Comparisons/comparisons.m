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

[initParams, infosAlgo] = loadDataFromADAT(pwd+pathDelimiter);


%% HEATMAP
% 
% heatMap = zeros(initParams.Na, initParams.dim(1), initParams.dim(2));
% 
% for e=1:initParams.epochDone
%     for n=1:initParams.Na
%         for k=1:infosAlgo(e, n).pathLength
%             pos = infosAlgo(e, n).finalPath(k, :);
%             heatMap(n, pos(1), pos(2)) = heatMap(n, pos(1), pos(2)) + 1;
%         end
%     end
% end
% 
% figure
% num = ceil(sqrt(initParams.Na));
% costs = initParams.costs;
% for i=1:initParams.Na
%     subplot(num, num, i)
%     colormap('hot')
%     imagesc(reshape(heatMap(i, :, :), [initParams.dim(1), initParams.dim(2)]))
%     colorbar
%     title({initParams.typeAlgoToStr(initParams.typeAlgo(i)), "Range="+num2str(initParams.ranges(i))+...
%         " Cost="+num2str(costs(i))})
% end
% xlabel(["",...
%     "Map Size: "+initParams.dim(1)+"x"+initParams.dim(2)+...
%     ", num obstacles: "+initParams.percNumObs+"%",...
%     "Start: ["+num2str(initParams.Sstart')+"], goal: ["+num2str(initParams.Sgoal')+"]",...
%     "Epochs done: "+initParams.epochDone])
% sgtitle("Heatmap")
% 
% 
% figure
% num = ceil(sqrt(initParams.Na));
% costs = initParams.costs;
% for i=1:initParams.Na
%     subplot(num, num, i)
%     colormap('hot')
%     appHeatMap = 1/(heatMap(i,:, :)+1);
%     imagesc(reshape(appHeatMap, [initParams.dim(1), initParams.dim(2)]))
%     colorbar
%     title({initParams.typeAlgoToStr(initParams.typeAlgo(i)), "Range="+num2str(initParams.ranges(i))+...
%         " Cost="+num2str(costs(i))})
% end
% xlabel(["",...
%     "Map Size: "+initParams.dim(1)+"x"+initParams.dim(2)+...
%     ", num obstacles: "+initParams.percNumObs+"%",...
%     "Start: ["+num2str(initParams.Sstart')+"], goal: ["+num2str(initParams.Sgoal')+"]",...
%     "Epochs done: "+initParams.epochDone])
% sgtitle("Inverse Heatmap")
% 
% figure
% num = ceil(sqrt(initParams.Na));
% costs = initParams.costs;
% for i=1:initParams.Na
%     subplot(num, num, i)
%     colormap('hot')
%     appHeatMap = (heatMap(i,:, :)>0);
%     imagesc(reshape(appHeatMap, [initParams.dim(1), initParams.dim(2)]))
%     colorbar
%     title({initParams.typeAlgoToStr(initParams.typeAlgo(i)), "Range="+num2str(initParams.ranges(i))+...
%         " Cost="+num2str(costs(i))})
% end
% xlabel(["",...
%     "Map Size: "+initParams.dim(1)+"x"+initParams.dim(2)+...
%     ", num obstacles: "+initParams.percNumObs+"%",...
%     "Start: ["+num2str(initParams.Sstart')+"], goal: ["+num2str(initParams.Sgoal')+"]",...
%     "Epochs done: "+initParams.epochDone])
% sgtitle("Traversed cells")
% 
% waitInput();

%% Comparisons
skip = [1, 2, 12, 32, 58, 67, 85,   43, 24, 81];
l_skip = length(skip);
initTimes4Epoch = zeros(initParams.epochDone-l_skip, initParams.Na);
computationTimes4Epoch = zeros(initParams.epochDone-l_skip, initParams.Na);
expCells4Epoch = zeros(initParams.epochDone-l_skip, initParams.Na);
totSteps4Epoch = zeros(initParams.epochDone-l_skip, initParams.Na);
pathLength4Epoch = zeros(initParams.epochDone-l_skip, initParams.Na);
continuousPathLength4Epoch = zeros(initParams.epochDone-l_skip, initParams.Na);

skipped = 0;
for i=1:initParams.epochDone
    if any(ismember(skip, i))
        skipped = skipped + 1;
        continue;
    end
    pos = i - skipped;
    for j=1:initParams.Na
        initTimes4Epoch(pos, j) = infosAlgo(i, j).initTime;
        computationTimes4Epoch(pos, j) = infosAlgo(i, j).replanningTime;
        expCells4Epoch(pos, j) = infosAlgo(i, j).expCells;
        totSteps4Epoch(pos, j) = infosAlgo(i, j).totSteps;
        pathLength4Epoch(pos, j) = infosAlgo(i, j).pathLength;
        continuousPathLength4Epoch(pos, j) = infosAlgo(i, j).continuousPathLength;
    end
end

%%
% figure
% bar(initTimes4Epoch)
% title("Initialization times")
% xlabel("Epoch")
% ylabel("Time (s)")
% grid on;
% legend(string(costs));
% 
% 
% figure
% bar(computationTimes4Epoch)
% title("Running time")
% xlabel("Epoch")
% ylabel("Time (s)")
% grid on;
% legend(string(costs));
% 
% figure
% bar(expCells4Epoch)
% title("Explored cells")
% xlabel("Epoch")
% ylabel("Number of cells")
% grid on;
% legend(string(costs));
% 
% figure
% bar(totSteps4Epoch)
% title("Total algorithm steps")
% xlabel("Epoch")
% ylabel("Number of steps")
% grid on;
% legend(string(costs));
% 
% figure
% bar(pathLength4Epoch)
% title("Path length")
% xlabel("Epoch")
% ylabel("Path length")
% grid on;
% legend(string(costs));
% 
% 
% figure
% subplot(1, 5, 1)
% bar(mean(initTimes4Epoch))
% title("Initialization time")
% grid on;
% xlabel("Algorithm")
% ylabel("Time (s)")
% 
% subplot(1, 5, 2)
% bar(mean(computationTimes4Epoch))
% title("Running time")
% xlabel("Algorithm")
% ylabel("Time (s)")
% grid on;
% 
% subplot(1, 5, 3)
% bar(mean(expCells4Epoch))
% title("Explored cells")
% xlabel("Algorithm")
% ylabel("Number of cells")
% grid on;
% 
% subplot(1, 5, 4)
% bar(mean(totSteps4Epoch))
% title("Total algorithm steps")
% xlabel("Algorithm")
% ylabel("Number of steps")
% grid on;
% 
% subplot(1, 5, 5)
% bar(mean(pathLength4Epoch))
% title("Path Length")
% xlabel("Algorithm")
% ylabel("Path length")
% grid on;
% 
% waitInput();

%%

figure
subplot(1, 5, 1)
boxplot(initTimes4Epoch)
title("Initialization time")
xlabel("Cost")
set(gca,'XTickLabel', [0.3, 0.6, 1, 5])
ylabel("Time (s)")
%ylim([0 8])
grid on;

subplot(1, 5, 2)
boxplot(computationTimes4Epoch)
title("Replanning time")
xlabel("Cost")
set(gca,'XTickLabel', [0.3, 0.6, 1, 5])
ylabel("Time (s)")
grid on;

subplot(1, 5, 3)
boxplot(expCells4Epoch)
title("Explored cells")
xlabel("Cost")
set(gca,'XTickLabel', [0.3, 0.6, 1, 5])
ylabel("Number of cells")
grid on;

subplot(1, 5, 4)
boxplot(totSteps4Epoch)
title("Total algorithm steps")
xlabel("Cost")
set(gca,'XTickLabel', [0.3, 0.6, 1, 5])
ylabel("Number of steps")
grid on;

subplot(1, 5, 5)
boxplot(pathLength4Epoch)
title("Path length")
xlabel("Cost")
set(gca,'XTickLabel', [0.3, 0.6, 1, 5])
ylabel("Path length")
grid on;


figure
subplot(1, 1, 1)
boxplot(continuousPathLength4Epoch)
title("Continuous Path length")
xlabel("Algorithm")
ylabel("Continuous Path length")
grid on;
% legend(flip(algos))

waitInput();

%% FUNCTIONS %%

function waitInput()
    disp("PAUSE: press enter to continue");
    pause();
    disp("CONTINUE!!");
end


