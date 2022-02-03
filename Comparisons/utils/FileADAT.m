classdef FileADAT < handle
    
    properties(Constant)
        ALGO_INIT_NO_ALGO = 0;
        
        ALGO_DSL_V1 = 1;
        ALGO_DSL_V2 = 2;
    end
    
    properties
        typeAlgo;
        
        dim;
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
            
            disp("Which algorithm?"+newline+...
                 "    1) D* Lite v1"+newline+...
                 "    2) D* Lite v2"+newline)
            typeOfAlgo = input('search option: ');
            switch(typeOfAlgo)
                case 1
                    obj.typeAlgo = FileADAT.ALGO_DSL_V1;
                    
                case 2
                    obj.typeAlgo = FileADAT.ALGO_DSL_V2;
                    
                otherwise
                    error("Wrong input!");
            end
            
            D1 = double(input("Input D1: "));
            D2 = double(input("Input D2: "));
            obj.dim = [D1; D2];
            
            obj.Sstart = [1; 1]; % TODO
            obj.Sgoal = [D1; D2]; % TODO
            obj.moves = [[1; 0], [1; 1], [0; 1], [-1; 1], [-1; 0], [-1; -1], [0; -1], [1; -1]];

            obj.Na = double(input("Input Na: "));

            obj.ranges = zeros(1, obj.Na);
            for i=1:obj.Na
                obj.ranges(i) = double(input("Input range i: "));
            end
            
            obj.costs = zeros(1, obj.Na);
            for i=1:obj.Na
                obj.costs(i) = double(input("Input costs i: "));
            end
        end
        
        function obj = constSTD()
            obj = FileADAT();
            
            obj.typeAlgo = FileADAT.ALGO_DSL_V1;
            
            D1 = 50;
            D2 = 50;
            obj.dim = [D1; D2];
            
            obj.Sstart = [1; 1];
            obj.Sgoal = [D1; D2];
            obj.moves = [[1; 0], [1; 1], [0; 1], [-1; 1], [-1; 0], [-1; -1], [0; -1], [1; -1]];

            obj.Na = 3;

            obj.ranges = [2   2   2];
            obj.costs  = [0.1 0.5 0.9];
        end
        
        function obj = getFromFile(fid)
            obj = FileADAT();
            
            obj.typeAlgo = str2double(regexp(fgetl(fid),'\d*','match')');
            % TODO check type
            
            fgetl(fid); % jump
            nums = str2double(regexp(fgetl(fid),'\d*','match')');
            obj.dim = nums(1:2);
            obj.Sstart = nums(3:4);
            obj.Sgoal = nums(5:6);
    
            nums = str2double(regexp(fgetl(fid),'[-]?\d*','match'));
            obj.moves = reshape(nums(3:end)', nums(1:2));
            
            obj.ranges = str2double(regexp(fgetl(fid),'\d+\.?\d*','match'));
            
            obj.costs = str2double(regexp(fgetl(fid),'\d+\.?\d*','match'));
            
            obj.Na = str2double(regexp(fgetl(fid),'\d*','match'));
            % TODO check lenght ranges costs and Na
            
            obj.epochDone = str2double(regexp(fgetl(fid),'\d*','match'));
            
            fgetl(fid); % jump
        end
    end
    
    methods
        function obj = FileADAT()
            obj.typeAlgo = FileADAT.ALGO_INIT_NO_ALGO;
            
            obj.dim = [];
            obj.Sstart = [];
            obj.Sgoal = [];
            
            obj.moves = [];
            obj.ranges = [];
            obj.costs = [];

            obj.Na = 0;
            obj.epochDone = 0;
        end
        
        function putOnFile(obj, fid)
            fprintf(fid, "typeAlgorithm %d\n", obj.typeAlgo);
            
            fprintf(fid, "mapSize_x mapSize_y start_x start_y goal_x goal_y\n");
            fprintf(fid, "%d %d %d %d %d %d\n", obj.dim, obj.Sstart, obj.Sgoal);
            
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
            e = (obj.typeAlgo == f.typeAlgo &&...
                all(obj.dim == f.dim) &&...
                all(obj.Sstart == f.Sstart) &&...
                all(obj.Sgoal == f.Sgoal) &&...
                all(all(obj.moves == f.moves)) &&...
                all(obj.ranges == f.ranges) &&...
                all(obj.costs == f.costs) &&...
                obj.Na == f.Na);
        end
    end
end