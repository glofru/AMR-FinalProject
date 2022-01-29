classdef AlgoInfo < handle
    properties
        expCells;
        expCellsList;
        totSteps;
        totStepsList;
        pathLenght;
    end
    
    methods
        function obj = AlgoInfo(expCells, expCellsList, totSteps, ...
                totStepsList, pathLenght)
            arguments
                expCells
                expCellsList
                totSteps
                totStepsList
                pathLenght
            end
                obj.expCells = expCells;
                obj.expCellsList = expCellsList;
                obj.totSteps = totSteps;
                obj.totStepsList = totStepsList;
                obj.pathLenght = pathLenght;
        end
    end
end