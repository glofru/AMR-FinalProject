function [initParams, infosAlgo] = loadDataFromADAT(inputPath, inputFile)
% Load the data from the file following the ADAT format.
%
% OUTPUT
%
% initParams: Cell contening the first k lines of the file
% NumRows: Number of the rows inside the body of the file
% NumCols: Number of the columns inside the body of the file
% m_fm: Matrix containig the data of the markers over the time

    arguments
        % Directory in which we are going to load the data as MOT file format
        inputPath string {mustBeFolder}
        % (OPTIONAL) Name of the file in which we want to write
        inputFile string {mustBeText} = ""
    end

    switch nargin
        case 0
            % if we haven't the file's name, we ask to the user to select it
            [inputFile, inputPath] = uigetfile("*.adat")
        case 1
            % if we haven't the file's name, we ask to the user to select it
            [inputFile, inputPath] = uigetfile(strcat(inputPath, "*.adat"));
        case 2
            % we have all the parameters, fine
        otherwise
            % we have too many parameters
            error('Wrong number of parameters!');
    end
    
    % open the file
    fid = fopen(strcat(inputPath, inputFile), 'rt');
    % write the first k lines from the header of the file
    initParams = FileADAT();
    
    for i=1:8
        line = fgetl(fid);
        initParams.getFromLine(i, line);
    end
    
    infosAlgo(initParams.epochDone, initParams.Na) = AlgoInfo();
    
    % preallocating memory for faster execution
    %m_fm = zeros(NumRows, NumCols);
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
end