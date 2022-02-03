function saveDataOnFileADAT(outputPath, initParams, infosAlgo, outputFile)
% Save the data inside the file following the ADAT format.

    arguments
        % Directory in which we are going to save the data as MOT file format
        outputPath string {mustBeFolder}
        % initParams, cell contening the first k lines of the file
        initParams %cell
        % Matrix containig the data over the time
        infosAlgo (:, :)
        % (OPTIONAL) Name of the file in which we want to write
        outputFile string {mustBeText} = ""
    end
    
    switch nargin
        case 3
            % if we haven't the file's name, we ask to the user to insert it
            outputFile = input("File Name: ", 's')+".adat";
        case 4
            % we have all the parameters, fine
        otherwise
            % we don't have enought parameters
            error('Wrong number of parameters!');
    end
    
    try
        disp("Saving in file: "+outputFile);
        tic;
        % open the file
        pathAndFile = outputPath+outputFile;

        if isfile(pathAndFile)
            [appInitParams, appInfosAlgo] = loadDataFromADAT(outputPath, outputFile);

            if ~(initParams == appInitParams)
                error("Non compatible initParams, change file name!")
            end

            old_e = appInitParams.epochDone;
            initParams.epochDone = initParams.epochDone + old_e;
            
            fid = fopen(pathAndFile, 'wt');
        else
            fid = fopen(pathAndFile, 'wt');
            old_e = 0;
            appInfosAlgo = [];
        end

        % write the first k lines for the heater
        initParams.putOnFile(fid);

        totWork = numel(appInfosAlgo)+numel(infosAlgo);

        ss = size(appInfosAlgo);
        reverseStr = '';
        for e=1:ss(1)
            for i=1:ss(2)
                fprintf(fid,'%d %d %s\n',e, i, jsonencode(appInfosAlgo(e, i)));

                % information print on the stdOut
                msg = sprintf('Percent done: %3.1f', 100 * (i+(e-1)*ss(2)) / totWork);
                fprintf([reverseStr, msg]);
                reverseStr = repmat(sprintf('\b'), 1, length(msg));
            end
        end

        ss = size(infosAlgo);
        for e=1:ss(1)
            for i=1:ss(2)
                fprintf(fid,'%d %d %s\n',e+old_e, i, jsonencode(infosAlgo(e, i)));

                % information print on the stdOut
                msg = sprintf('Percent done: %3.1f', 100 * (i+(e-1)*ss(2)+numel(appInfosAlgo)) / totWork);
                fprintf([reverseStr, msg]);
                reverseStr = repmat(sprintf('\b'), 1, length(msg));
            end
        end

        % close the file
        fclose(fid);
        
        disp(" ");
        disp("Saving done in: <strong>"+string(toc)+...
            "</strong> s");
        
    catch ME
        fclose(fid);
        rethrow(ME);
    end
end