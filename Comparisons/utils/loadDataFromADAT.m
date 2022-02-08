function [initParams, infosAlgo] = loadDataFromADAT(inputPath, inputFile)
% Load the data from the file following the ADAT format.
%
% OUTPUT
%
% initParams: Cell contening the first k lines of the file
% infosAlgo: Matrix containig the data of the algos

    arguments
        % Directory in which we are going to load the data as ADAT file format
        inputPath string {mustBeFolder}
        % (OPTIONAL) Name of the file in which we want to write
        inputFile string {mustBeText} = ""
    end

    switch nargin
        case 0
            % if we haven't the file's name, we ask to the user to select it
            [inputFile, inputPath] = uigetfile("*.adat");
        case 1
            % if we haven't the file's name, we ask to the user to select it
            [inputFile, inputPath] = uigetfile(strcat(inputPath, "*.adat"));
        case 2
            % we have all the parameters, fine
        otherwise
            % we have too many parameters
            error('Wrong number of parameters!');
    end
    
    try
        disp("Loading from file: "+inputFile);
        tic;
        % open the file
        fid = fopen(strcat(inputPath, inputFile), 'rt');
        % write the first k lines from the header of the file
        initParams = FileADAT.getFromFile(fid);

        infosAlgo(initParams.epochDone, initParams.Na) = AlgoInfo();

        % preallocating memory for faster execution
        totWork = numel(infosAlgo);
        reverseStr = '';
        for i=1:totWork
            nums = fscanf(fid, '%f %f');
            jc = fgetl(fid);
            infosAlgo(nums(1), nums(2)) = AlgoInfo.createFromStruct(jsondecode(jc));

            % information print on the stdOut
            msg = sprintf('Percent done: %3.1f', 100 * i / totWork);
            fprintf([reverseStr, msg]);
            reverseStr = repmat(sprintf('\b'), 1, length(msg));
        end
        % close the file
        fclose(fid);
        
        disp(" ");
        disp("Loading done in: <strong>"+string(toc)+...
            "</strong> s");
        
    catch ME
        fclose(fid);
        rethrow(ME);
    end
end