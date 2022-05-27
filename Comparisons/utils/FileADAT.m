classdef FileADAT < handle
    
    properties(Constant)
        ALGO_INIT_NO_ALGO = 0;
        
        ALGO_DS = 1;
        ALGO_DSL_V1 = 2;
        ALGO_DSL_V2 = 3;
        ALGO_FDS = 4;
        
        ALGO_CUSTOM = 9;
        ALGO_ALL = 10;
    end
    
    properties
        typeAlgo;
        
        dim;
        percNumObs;
        Sstart;
        Sgoal;
        
        moves;
        ranges;
        costs;
        
        Na;
        epochDone;
    end
    
    methods(Static)
        function obj = constByUser()
            obj = FileADAT();
            
            obj.typeAlgo = FileADAT.selectAlgoType();
            
            D1 = double(input("Input D1: "));
            D2 = double(input("Input D2: "));
            obj.dim = [D1; D2];
            
            obj.percNumObs = double(input("Percentage number of obstacles: "));
            
            obj.Sstart = [1; 1]; % TODO
            obj.Sgoal = [D1; D2]; % TODO
            obj.moves = [[1; 0], [1; 1], [0; 1], [-1; 1], [-1; 0], [-1; -1], [0; -1], [1; -1]];

            if obj.typeAlgo == FileADAT.ALGO_ALL
                obj.Na = 4;
                obj.typeAlgo = [FileADAT.ALGO_DS, FileADAT.ALGO_DSL_V1, FileADAT.ALGO_DSL_V2, FileADAT.ALGO_FDS];
            elseif obj.typeAlgo == FileADAT.ALGO_CUSTOM
                obj.Na = double(input("Number of algorithms: "));
                
                for i=1:obj.Na
                    obj.typeAlgo(i) = FileADAT.selectAlgoTypeRestricted();
                end
            else
                obj.Na = double(input("Number of tries: "));
                obj.typeAlgo = ones(1, obj.Na) * obj.typeAlgo;
            end
            
            obj.ranges = zeros(1, obj.Na);
            obj.costs = zeros(1, obj.Na);
            
            for i=1:obj.Na
                obj.ranges(i) = double(input("Input range try N. " + i + ": "));
                obj.costs(i) = double(input("Input costs try N. " + i + ": "));
            end
        end
        
        function obj = constSTD()
            obj = FileADAT();
            
            obj.typeAlgo = FileADAT.selectAlgoType();
            
            if obj.typeAlgo == FileADAT.ALGO_ALL || obj.typeAlgo == FileADAT.ALGO_CUSTOM
                if obj.typeAlgo == FileADAT.ALGO_CUSTOM
                    obj.Na = double(input("Number of algorithms: "));
                
                    for i=1:obj.Na
                        obj.typeAlgo(i) = FileADAT.selectAlgoTypeRestricted();
                    end
                else
                    obj.Na = 4;
                    obj.typeAlgo = [FileADAT.ALGO_DS, FileADAT.ALGO_DSL_V1, FileADAT.ALGO_DSL_V2, FileADAT.ALGO_FDS];
                end
                
                obj.ranges = ones(1, obj.Na) * 2;
                obj.costs  = ones(1, obj.Na) * 0.6;
            else
                obj.Na = 4;
                obj.typeAlgo = ones(1, obj.Na) * obj.typeAlgo;
                
                obj.ranges = ones(1, obj.Na) * 3;
                obj.costs  = [0.3, 0.6, 1, 5];
            end
            
            D1 = 40;
            D2 = 40;
            obj.percNumObs = 40;
            obj.dim = [D1; D2];
            
            obj.Sstart = [1; 1];
            obj.Sgoal = [D1; D2];
            obj.moves = [[1; 0], [1; 1], [0; 1], [-1; 1], [-1; 0], [-1; -1], [0; -1], [1; -1]];
        end
        
        function obj = getFromFile(fid)
            obj = FileADAT();
            
            obj.typeAlgo = str2double(regexp(fgetl(fid),'\d*','match'));
            % TODO check type
            
            fgetl(fid); % jump
            nums = str2double(regexp(fgetl(fid),'\d*','match')');
            obj.dim = nums(1:2);
            obj.percNumObs = nums(3);
            obj.Sstart = nums(4:5);
            obj.Sgoal = nums(6:7);
    
            nums = str2double(regexp(fgetl(fid),'[-]?\d*','match'));
            obj.moves = reshape(nums(3:end)', nums(1:2));
            
            obj.ranges = str2double(regexp(fgetl(fid),'\d+\.?\d*','match'));
            
            obj.costs = str2double(regexp(fgetl(fid),'\d+\.?\d*','match'));
            
            obj.Na = str2double(regexp(fgetl(fid),'\d*','match'));
            % TODO check length ranges costs and Na
            
            obj.epochDone = str2double(regexp(fgetl(fid),'\d*','match'));
            
            fgetl(fid); % jump
        end
        
        function at = selectAlgoType()
            disp("Which algorithm?"+newline+...
                 "    1) D*"+newline+...
                 "    2) D* Lite v1"+newline+...
                 "    3) D* Lite v2"+newline+...
                 "    4) Field D*"+newline+...
                 "    5) Custom"+newline+...
                 "    6) All")
            typeOfAlgo = input('search option: ');
            switch(typeOfAlgo)
                case 1
                    at = FileADAT.ALGO_DS;
                case 2
                    at = FileADAT.ALGO_DSL_V1;
                case 3
                    at = FileADAT.ALGO_DSL_V2;
                case 4
                    at = FileADAT.ALGO_FDS;
                case 5
                    at = FileADAT.ALGO_CUSTOM;
                case 6
                    at = FileADAT.ALGO_ALL;
                otherwise
                    error("Wrong input!");
            end
        end
        function at = selectAlgoTypeRestricted()
            disp("Which algorithm?"+newline+...
                 "    1) D*"+newline+...
                 "    2) D* Lite v1"+newline+...
                 "    3) D* Lite v2"+newline+...
                 "    4) Field D*")
            typeOfAlgo = input('search option: ');
            switch(typeOfAlgo)
                case 1
                    at = FileADAT.ALGO_DS;
                case 2
                    at = FileADAT.ALGO_DSL_V1;
                case 3
                    at = FileADAT.ALGO_DSL_V2;
                case 4
                    at = FileADAT.ALGO_FDS;
                otherwise
                    error("Wrong input!");
            end
        end
    end
    
    methods
        function obj = FileADAT()
            obj.typeAlgo = FileADAT.ALGO_INIT_NO_ALGO;
            
            obj.dim = [];
            obj.percNumObs = 0;
            obj.Sstart = [];
            obj.Sgoal = [];
            
            obj.moves = [];
            obj.ranges = [];
            obj.costs = [];

            obj.Na = 0;
            obj.epochDone = 0;
        end
        
        function putOnFile(obj, fid)
            ftm = [' ', repmat('%g ', 1, numel(obj.typeAlgo)-1), '%g\n'];
            fprintf(fid, "typeAlgorithm "+ftm, obj.typeAlgo);
            
            fprintf(fid, "mapSize_x mapSize_y percNumObs start_x start_y goal_x goal_y\n");
            fprintf(fid, "%d %d %d %d %d %d %d\n", obj.dim, obj.percNumObs, obj.Sstart, obj.Sgoal);
            
            ftm = ['moves %d %d ', repmat('%g ', 1, numel(obj.moves)-1), '%g\n'];
            fprintf(fid, ftm, size(obj.moves, 1), size(obj.moves, 2), obj.moves);
            
            ftm = [' ', repmat('%g ', 1, numel(obj.ranges)-1), '%g\n'];
            fprintf(fid, "ranges"+ftm, obj.ranges);
            fprintf(fid, "costs"+ftm, obj.costs);
            
            fprintf(fid, "Na %d\n", obj.Na);
            fprintf(fid, "epochDone %d\n", obj.epochDone);
            fprintf(fid, "epoch Nalgo jsonencode\n");
        end
        
        function e = eq(obj, f)
            e = (all(obj.typeAlgo == f.typeAlgo) &&...
                all(obj.dim == f.dim) &&...
                obj.percNumObs == f.percNumObs &&...
                all(obj.Sstart == f.Sstart) &&...
                all(obj.Sgoal == f.Sgoal) &&...
                all(all(obj.moves == f.moves)) &&...
                all(obj.ranges == f.ranges) &&...
                all(obj.costs == f.costs) &&...
                obj.Na == f.Na);
        end

        function s = typeAlgoToStr(obj, typeAlgo)
            switch typeAlgo
                case FileADAT.ALGO_DS
                    s = "D*";
                case FileADAT.ALGO_DSL_V1
                    s = "D* Lite v1";
                case FileADAT.ALGO_DSL_V2
                    s = "D* Lite v2";
                case FileADAT.ALGO_FDS
                    s = "Field D*";
                otherwise
                    s = "";
            end
        end
    end
end


