classdef AlgoInfo < handle
    properties
        initTime;
        computationTime;
        
        finalPath;
        
        expCells;
        expCellsList;
        totSteps;
        totStepsList;
        pathLenght;
    end
    
    methods(Static)
        function obj = createFromStruct(s)
            obj = AlgoInfo();
            obj.initTime = s.initTime;
            obj.computationTime = s.computationTime;
            obj.finalPath = s.finalPath;
            obj.expCells = s.expCells;
            obj.expCellsList = s.expCellsList;
            obj.totSteps = s.totSteps;
            obj.totStepsList = s.totStepsList;
            obj.pathLenght = s.pathLenght;
       end
    end
    
    methods
        function obj = AlgoInfo()
            obj.initTime = 0;
            obj.computationTime = 0;
            obj.finalPath = [];
            obj.expCells = 0;
            obj.expCellsList = [];
            obj.totSteps = 0;
            obj.totStepsList = [];
            obj.pathLenght = 0;
        end
    end
end