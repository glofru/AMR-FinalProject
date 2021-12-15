classdef PriorityQueue
    properties (Access = private)
        %
        queueS;
        %
        queueK;
    end
    
    methods (Access = private)
        
    end
    
    methods (Access = public)
        function obj = PriorityQueue()
            % constructor
            obj.queueS = [];
            obj.queueK = [];
        end
        
        function obj = insert(obj, s, k)
            % insert in the queue vertex s with valur k
            obj.queueS(:, end+1) = s;
            obj.queueK(:, end+1) = k;
        end
        
        function pos = find(obj, s)
            % find in the queue vertex s
            % if not exists return -1
            pos = -1;
            for i=1:size(obj.queueS, 2)
                if all(s==obj.queueS(:, i))
                    pos=i;
                    return;
                end
            end
        end
        
        function res = isEmpty(obj)
            res = isempty(obj.queueS);
        end
        
        function isInside = has(obj, s)
            % check if the queue has vertex s
            
            isInside = true;
            if obj.find(s) == -1
                isInside = false;
            end
        end
        
        function obj = remove(obj, s)
            % remove from the queue vertex s
            pos = obj.find(s);
            
            obj.queueS(:, pos) = [];
            obj.queueK(:, pos) = [];
        end
        
        function [minS, minV] = top(obj)
            % return a vertex with the smallest priority k
            % optional minV
            
            minV = [];
            minS = [];
            for i=1:size(obj.queueS, 2)
                appMinV = obj.queueK(:, i); % TODO
                if isempty(minV) || min2(appMinV, minV)
                    minV = obj.queueK(:, i);
                    minS = obj.queueS(:, i);
                end
            end
        end
        
        function minV = topKey(obj)
            % return the smallest priority k of all vertices
            % if empty return [+inf, +inf]
            minV = [];
            for i=1:size(obj.queueS, 2)
                if isempty(minV) || min2(obj.queueK(:, i), minV)
                    minV = obj.queueK(:, i);
                end
            end
            
            if isempty(minV)
                error("the priority queue is empty");
            end
        end
        
        function [obj, s, k] = pop(obj)
            % delete from the queue the vertex with the smallest priority k
            % and return it
            % optional k
            [s, k] = top(obj);
            obj = obj.remove(s);
        end
        
        function obj = update(obj, s, k)
            % change priority of s from k (old) to k
            pos = obj.find(s);
            if pos == -1
                obj = obj.insert(s, k);
            else
                obj.queueK(:, pos) = k; 
            end
        end
    end
    
    % plot functions
    methods (Access = public)
        function plotPriorityQueue(obj)
            disp("Priority Queue:")
            data = [obj.queueS; obj.queueK];
            for col=data
                disp(strjoin(["    [", col(1:2)', "] --> [", col(3:4)', "]"]))
            end
        end
    end
end


