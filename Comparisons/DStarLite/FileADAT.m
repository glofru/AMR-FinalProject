classdef FileADAT < handle
    properties
        dim;
        Sstart;
        Sgoal;
        
        moves;
        ranges;
        costs;
        
        Na;
        epochDone;
    end
    
    methods
        function obj = FileADAT()
            obj.dim = [];
            obj.Sstart = [];
            obj.Sgoal = [];
            
            obj.moves = [];
            obj.ranges = [];
            obj.costs = [];

            obj.Na = 0;
            obj.epochDone = 0;
        end
        
        function getFromLine(obj, pos, line)
            switch(pos)
                case 1
                    return; % jump
                case 2
                    nums = str2double(regexp(line,'\d*','match')');
                    obj.dim = nums(1:2);
                    obj.Sstart = nums(3:4);
                    obj.Sgoal = nums(5:6);
                case 3
                    nums = str2double(regexp(line,'[-]?\d*','match'));
                    obj.moves = reshape(nums(3:end)', nums(1:2));
                case 4
                    obj.ranges = str2double(regexp(line,'\d+\.?\d*','match'));
                case 5
                    obj.costs = str2double(regexp(line,'\d+\.?\d*','match'));
                case 6
                    obj.Na = str2double(regexp(line,'\d*','match'));
                    % TODO check lenght ranges costs and Na
                case 7
                    obj.epochDone = str2double(regexp(line,'\d*','match'));
                case 8
                    return; % jump
            end
        end
        
        function putOnFile(obj, fid)
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
    end
end