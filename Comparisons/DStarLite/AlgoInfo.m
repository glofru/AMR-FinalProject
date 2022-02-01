classdef AlgoInfo < handle
    properties
        initTime;
        computationTime;
        
        % add heatmap
        
        expCells;
        expCellsList;
        totSteps;
        totStepsList;
        pathLenght;
    end
    
    methods(Static)
        function obj = createFromStruct(struct)
            obj = AlgoInfo();
            obj.initTime = struct.initTime;
            obj.computationTime = struct.computationTime;
            obj.expCells = struct.expCells;
            obj.expCellsList = struct.expCellsList;
            obj.totSteps = struct.totSteps;
            obj.totStepsList = struct.totStepsList;
            obj.pathLenght = struct.pathLenght;
       end
    end
    
    methods
        function obj = AlgoInfo()
            if nargin > 0
                obj.Value = v;
            end
            obj.initTime = 0;
            obj.computationTime = 0;
            obj.expCells = 0;
            obj.expCellsList = [];
            obj.totSteps = 0;
            obj.totStepsList = [];
            obj.pathLenght = 0;
        end
    end
end