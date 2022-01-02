classdef Greedy_best_first < handle
    %RRT_PRIMITIVES Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        map_limit
        goal
        map
        start
        cells_isopen
        size_x
        size_y
        resolution
        %cells_closed
     
    end
    
    methods
        function obj = Greedy_best_first(initial_state,sampling_time,limit,goal,map,resolution,maxIter)
            %   Detailed explanation goes here

            obj.map_limit = limit;
            obj.goal = int16(goal/resolution);
            obj.start = [int16(initial_state(1)/resolution) int16(initial_state(2)/resolution)];
            obj.resolution = resolution;
            %obj.map = im2double(map);            
            %[x, y, g, h, open, parent id (the position in the array)]
            obj.map = zeros(size(map,1)*size(map,2),6);
            obj.size_x = size(map,1);
            obj.size_y = size(map,2);
            %[open, cost]
            obj.cells_isopen = zeros(size(map,1)*size(map,2),2);
            %obj.cells_closed = zeros(size(map,1)*size(map,2),1);
            
            for i = 1:size(map,1)
               for j = 1:size(map,2) 
                  if(map(i,j) < 250) 
                      obj.cells_isopen(i +(j-1)*size(map,2),:) = [-1,10000];
                  else
                      %h = abs(obj.goal(1)-((i-1)*resolution)) + abs(obj.goal(2)-((j-1)*resolution));
                      h = 0;
                      obj.map(i +(j-1)*size(map,2),:) = [i,j,0,h,0,0];
                      if(i == obj.start(1) & j == obj.start(2))
                        obj.cells_isopen(i + (j-1)*size(map,2),:) = [1,0];
                      else
                        obj.cells_isopen(i + (j-1)*size(map,2),:) = [0,10000];
                      end
                  end
               end
            end
        end
        
        function best_cell = find_next_cell(obj)
            best_cost = 10000;
            best_cell = -1;
            for i = 1:obj.size_x
               for j = 1:obj.size_y
                  index = i+(j-1)*obj.size_y;
                  if(obj.cells_isopen(index,1) == 1 & obj.cells_isopen(index,2) <= best_cost)
                     best_cost = obj.cells_isopen(index,2);
                     best_cell = index;
                  end
               end
            end      

        end

        function open_sons_cell(obj,parent_index)
            x = obj.map(parent_index,1);
            y = obj.map(parent_index,2);
            cost_parent = obj.cells_isopen(parent_index,2);
            obj.cells_isopen(parent_index,1) = -1;
            

            %need to check 8 sons!
            for i=-1:1
               for j=-1:1   
                    if(i == 0 & j == 0)
                        continue;
                    end
                    x_new = x + i; y_new = y + j;
                    if(x_new*obj.resolution <= obj.map_limit(1) & y_new*obj.resolution <= obj.map_limit(2))
                       %check if is open or closed!
                       
                       %calculate cost 
                       %graph_cost = 1;               
                       %new_cost = cost_parent+graph_cost;
                       son_index = x_new+(y_new-1)*obj.size_y;

                       if(obj.cells_isopen(son_index,1) ~= -1 & obj.cells_isopen(son_index,1) == 0)
                           %if new_cost < obj.cells_isopen(son_index,2)
                               h = abs(obj.goal(1)-(x_new)) + abs(obj.goal(2)-(y_new));
                               %h = sqrt((obj.goal(1)*obj.resolution-(x_new*obj.resolution))^2 + (obj.goal(2)*obj.resolution-(y_new*obj.resolution))^2);
                               new_cost = h; 
                               %new_cost
                               obj.cells_isopen(son_index,2) = new_cost;
                               obj.map(son_index,6) = parent_index;
                               obj.cells_isopen(son_index,1) = 1;

                           %end
                       end
                    end
                end
            end    
        end
     
        
        function finish = check_goal(obj,index)
            x = obj.map(index,1);
            y = obj.map(index,2);
            if(x == obj.goal(1) & y == obj.goal(2))
                finish = 1;
            else
                finish = 0;
            end
        end
        
        function path = take_path(obj)
            index = obj.goal(1) + (obj.goal(2)-1)*obj.size_y;
            path = ones(100,6);
            dimension_path = 0;
            for i=1:100
                path(i,1:2) = obj.map(index,1:2)*obj.resolution; 
                index = obj.map(index,6);
                dimension_path = dimension_path + 1;
                if(index == 0)
                    break;
                end
            end
            path = path(1:dimension_path,:);
        end
        
        %end methods
    end
        

%end class
end

