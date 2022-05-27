classdef AlgoInfo < handle
    properties
        initTime;
        replanningTime;
        
        finalPath;
        
        expCells;
        expCellsList;
        totSteps;
        totStepsList;
        pathLength;
        continuousPathLength
    end
    
    methods(Static)
        function obj = createFromStruct(s)
            obj = AlgoInfo();
            obj.initTime = s.initTime;
            obj.replanningTime = s.replanningTime;
            obj.finalPath = s.finalPath;
            obj.expCells = s.expCells;
            obj.expCellsList = s.expCellsList;
            obj.totSteps = s.totSteps;
            obj.totStepsList = s.totStepsList;
            obj.pathLength = s.pathLength;
            obj.continuousPathLength = s.continuousPathLength;
       end
    end
    
    methods
        function obj = AlgoInfo()
            obj.initTime = 0;
            obj.replanningTime = 0;
            obj.finalPath = [];
            obj.expCells = 0;
            obj.expCellsList = [];
            obj.totSteps = 0;
            obj.totStepsList = [];
            obj.pathLength = 0;
            obj.continuousPathLength = 0;
        end
    end
end


