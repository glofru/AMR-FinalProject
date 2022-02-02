clear all
close all
clc
restoredefaultpath

addpath('./DStarLite')

%%

% a = AlgoInfo();
% disp(a);
% a.initTime = 10;
% disp(a);
% 
% jc = jsonencode(a)
% b = jsondecode(jc)

%%
dim = 500;

tic
infosAlgo(dim, dim) = AlgoInfo();
toc
% dim = 500 --> Elapsed time is 2.415102 seconds. --> 64.4448 times faster
% dim = 1000 --> Elapsed time is 8.111521 seconds.

tic
infosAlgo = AlgoInfo.empty(1, 0);
for i=1:dim
    tmp = AlgoInfo.empty(0, 1);
     for j=1:dim
        tmp(j) = AlgoInfo();
    end
    infosAlgo = [infosAlgo; tmp];
end
toc
% dim = 500 --> Elapsed time is 155.640691 seconds.
% dim = 1000 --> Elapsed time is almost Inf seconds.

%% LOAD DATA
warning('error', 'MATLAB:deblank:NonStringInput');
%inputPath = strcat(uigetdir('', 'Select Input Directory'), '\');

inputPath = 'D:\Università\Magistrale La Sapienza\2 Primo semestre\Autonomous and Mobile Robotics\AMR-FinalProject\Comparisons\';
inputFile = 'test1.adat';
outputFile = 'test2.adat';

tic
[initParams, infosAlgo] = loadDataFromADAT(inputPath, inputFile);
toc

% tic
% saveDataOnFileADAT(inputPath, initParams, infosAlgo, outputFile);
% toc
