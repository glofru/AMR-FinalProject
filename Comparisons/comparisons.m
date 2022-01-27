clear all;
clc

%% Main

disp("Which search algorithm?"+newline+...
     "    1) D*"+newline+...
     "    2) D*Lite v1"+newline+...
     "    3) D*Lite v2"+newline+...
     "    4) Field D*"+newline)
algorithmType = 2; % input('search algorithm: ');

D1 = 25; % 25
D2 = 50; % 50
dim = [D1; D2];
Sstart = [1; 1];
Sgoal = [D1; D2];

moves = [[1; 0], [1; 1], [0; 1], [-1; 1], [-1; 0], [-1; -1], [0; -1], [1; -1]];
costs = [0.1, 1, 2];

execute = true;
while execute

    globalObstacles = zeros(2, round(D1*D2/2));
    for i=1:round(D1*D2/2)
        x = round(mod(rand*D1, D1))+1;
        y = round(mod(rand*D2, D2))+1;

        % obstacles overlap, ok, not an error
        if ~(all([x; y]==Sstart) || all([x; y]==Sgoal))
            globalObstacles(:, i) = [x; y];
        end
    end

    

    switch algorithmType
        case 1


        case 2
            % DEF MAP
            tic
            addpath('./DStarLite')
            map = Map(dim(1), dim(2), globalObstacles, Map.TYPE_KNOWN, 1);
            map.map(Sstart(1), Sstart(2)).state = Map.MAP_START;
            map.map(Sgoal(1), Sgoal(2)).state = Map.MAP_GOAL;

            img_g = map.buildImageMap();

            % INIT ALGORITHM
            knownObstacles = [];
            algorithm_c1 = D_star_lite_v1(map, knownObstacles, Sstart, Sgoal, moves, costs(1));
            algorithm_c2 = D_star_lite_v1(map, knownObstacles, Sstart, Sgoal, moves, costs(2));
            algorithm_c3 = D_star_lite_v1(map, knownObstacles, Sstart, Sgoal, moves, costs(3));
            
            plotAlgs(img_g, algorithm_c1, algorithm_c2, algorithm_c3);
            
            disp('Inizialization terminated in: '+string(toc)+' s'+newline);
            disp("PAUSE: press enter to continue");pause()

            % RUN ALGORITHM
            tic
            finised = [0, 0, 0];
            while ~all(finised)
                if algorithm_c1.isFinish()
                    finised(1) = 1;
                else
                    algorithm_c1.step();
                end
                
                if algorithm_c2.isFinish()
                    finised(2) = 1;
                else
                    algorithm_c2.step();
                end
                
                if algorithm_c3.isFinish()
                    finised(3) = 1;
                else
                    algorithm_c3.step();
                end

                plotAlgs(img_g, algorithm_c1, algorithm_c2, algorithm_c3);
            end
            
            disp('run terminated in: '+string(toc)+' s'+newline);

        case 3

            
        case 4


        otherwise
            error("Wrong!");
    end
    
    execute = input("Another map? [0=No/1=Yes] ");
    try
        if execute ~= 0 && execute ~= 1
            execute = 0;
        end
    catch
        execute = 0;
    end
    
end
disp("Terminated!")

%% FUNCTIONS

function plotAlgs(img_g, algorithm_c1, algorithm_c2, algorithm_c3)
    img_c1 = algorithm_c1.localMap.buildImageMap();
    img_c2 = algorithm_c2.localMap.buildImageMap();
    img_c3 = algorithm_c3.localMap.buildImageMap();
    
    ax1 = subplot(3,3,4);
    image(ax1, img_g);
    title(ax1, "Global Map");
    axis off;
    

    % ### 1 ###
    ax2 = subplot(3,3,2);
    image(ax2, img_c1);
    title(ax2, {"Algorithm Map c=0.1";...
        "explored cells: "+num2str(algorithm_c1.getExpCells())+...
        ", tot steps: "+num2str(algorithm_c1.getTotSteps())+...
        ", path lenght: "+num2str(algorithm_c1.getPathLenght())
        })
    axis off;
    
    ax2_2 = subplot(3,3,3);
    title(ax2_2, 'Algorithm c=0.1')
%     yyaxis left
%     ylabel(ax2_2, 'explored cells')
%     y_axis1 = diff(algorithm_c1.expCellsList)';
%     bar(ax2_2, [ y_axis1, zeros(size(y_axis1)) ])
    yyaxis right
    ylabel(ax2_2, 'work for step')
    y_axis2 = diff(algorithm_c1.totStepsList)';
    bar(ax2_2, [ zeros(size(y_axis2)) ,y_axis2])

    
    % ### 2 ###
    ax3 = subplot(3,3,5);
    image(ax3, img_c2);
    title(ax3, {"Algorithm Map c=1";...
        "explored cells: "+num2str(algorithm_c2.getExpCells())+...
        ", tot steps: "+num2str(algorithm_c2.getTotSteps())+...
        ", path lenght: "+num2str(algorithm_c2.getPathLenght())
        })
    axis off;
    
    ax3_2 = subplot(3,3,6);
    title(ax3_2, 'Algorithm c=1')
%     yyaxis left
%     ylabel(ax3_2, 'explored cells')
%     y_axis1 = diff(algorithm_c2.expCellsList)';
%     bar(ax3_2, [ y_axis1, zeros(size(y_axis1)) ])
    yyaxis right
    ylabel(ax3_2, 'work for step')
    y_axis2 = diff(algorithm_c2.totStepsList)';
    bar(ax3_2, [ zeros(size(y_axis2)) ,y_axis2])

    
    % ### 3 ###
    ax4 = subplot(3,3,8);
    image(ax4, img_c3);
    title(ax4, {"Algorithm Map c=2";...
        "explored cells: "+num2str(algorithm_c3.getExpCells())+...
        ", tot steps: "+num2str(algorithm_c3.getTotSteps())+...
        ", path lenght: "+num2str(algorithm_c3.getPathLenght())
        })
    axis off;
    
    ax4_2 = subplot(3,3,9);
    title(ax4_2, 'Algorithm c=2')
%     yyaxis left
%     ylabel(ax4_2, 'explored cells')
%     y_axis1 = diff(algorithm_c3.expCellsList)';
%     bar(ax4_2, [ y_axis1, zeros(size(y_axis1)) ])
    yyaxis right
    ylabel(ax4_2, 'work for step')
    y_axis2 = diff(algorithm_c3.totStepsList)';
    bar(ax4_2, [ zeros(size(y_axis2)) ,y_axis2])
    
    pause(0.5)
end

function WIP_plotAlgs(img_g, algorithm_c1, algorithm_c2, algorithm_c3)
    img_c1 = algorithm_c1.localMap.buildImageMap();
    img_c2 = algorithm_c2.localMap.buildImageMap();
    img_c3 = algorithm_c3.localMap.buildImageMap();
    

    % ### 1 ###
    ax2 = subplot(3,2,1);
    image(ax2, img_c1);
    title(ax2, {"Algorithm Map c=0.1";...
        "explored cells: "+num2str(algorithm_c1.getExpCells())+...
        ", tot steps: "+num2str(algorithm_c1.getTotSteps())+...
        ", path lenght: "+num2str(algorithm_c1.getPathLenght())
        })
    axis off;
    
    ax2_2 = subplot(3,2,2);
    title(ax2_2, 'Algorithm c=0.1')
    yyaxis left
    ylabel(ax2_2, 'explored cells')
    y_axis1 = diff(algorithm_c1.expCellsList)';
    bar(ax2_2, [ y_axis1, zeros(size(y_axis1)) ])
    yyaxis right
    ylabel(ax2_2, 'work for step')
    y_axis2 = diff(algorithm_c1.totStepsList)';
    bar(ax2_2, [ zeros(size(y_axis2)) ,y_axis2])

    
    % ### 2 ###
    ax3 = subplot(3,2,3);
    image(ax3, img_c2);
    title(ax3, {"Algorithm Map c=1";...
        "explored cells: "+num2str(algorithm_c2.getExpCells())+...
        ", tot steps: "+num2str(algorithm_c2.getTotSteps())+...
        ", path lenght: "+num2str(algorithm_c2.getPathLenght())
        })
    axis off;
    
    ax3_2 = subplot(3,2,4);
    title(ax3_2, 'Algorithm c=1')
    yyaxis left
    ylabel(ax3_2, 'explored cells')
    y_axis1 = diff(algorithm_c2.expCellsList)';
    bar(ax3_2, [ y_axis1, zeros(size(y_axis1)) ])
    yyaxis right
    ylabel(ax3_2, 'work for step')
    y_axis2 = diff(algorithm_c2.totStepsList)';
    bar(ax3_2, [ zeros(size(y_axis2)) ,y_axis2])

    
    % ### 3 ###
    ax4 = subplot(3,2,5);
    image(ax4, img_c3);
    title(ax4, {"Algorithm Map c=2";...
        "explored cells: "+num2str(algorithm_c3.getExpCells())+...
        ", tot steps: "+num2str(algorithm_c3.getTotSteps())+...
        ", path lenght: "+num2str(algorithm_c3.getPathLenght())
        })
    axis off;
    
    ax4_2 = subplot(3,2,6);
    title(ax4_2, 'Algorithm c=2')
    yyaxis left
    ylabel(ax4_2, 'explored cells')
    y_axis1 = diff(algorithm_c3.expCellsList)';
    bar(ax4_2, [ y_axis1, zeros(size(y_axis1)) ])
    yyaxis right
    ylabel(ax4_2, 'work for step')
    y_axis2 = diff(algorithm_c3.totStepsList)';
    bar(ax4_2, [ zeros(size(y_axis2)) ,y_axis2])
    
    
    y_axis1 = diff(algorithm_c1.totStepsList)';
    y_axis2 = diff(algorithm_c2.totStepsList)';
    y_axis3 = diff(algorithm_c3.totStepsList)';
    maxDim = max([length(y_axis1), length(y_axis2), length(y_axis3)]);
    y_addon1 = zeros(maxDim-length(y_axis1), 1);
    y_addon2 = zeros(maxDim-length(y_axis2), 1);
    y_addon3 = zeros(maxDim-length(y_axis3), 1);
    
    bar([[y_axis1; y_addon1], [y_axis2; y_addon2], [y_axis3; y_addon3]])
    
    
    
    y_axis1 = diff(algorithm_c1.expCellsList)';
    y_axis2 = diff(algorithm_c2.expCellsList)';
    y_axis3 = diff(algorithm_c3.expCellsList)';
    maxDim = max([length(y_axis1), length(y_axis2), length(y_axis3)]);
    y_addon1 = zeros(maxDim-length(y_axis1), 1);
    y_addon2 = zeros(maxDim-length(y_axis2), 1);
    y_addon3 = zeros(maxDim-length(y_axis3), 1);
    
    bar([[y_axis1; y_addon1], [y_axis2; y_addon2], [y_axis3; y_addon3]])
    
    pause(0.5)
end
