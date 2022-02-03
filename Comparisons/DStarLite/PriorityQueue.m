classdef PriorityQueue
    properties %(Access = private)
        %
        queue;
    end
    
    methods (Access = private)
        
    end
    
    methods (Access = public)
        function obj = PriorityQueue()
            % constructor
            obj.queue = State.empty(1, 0);
        end
        
        function obj = insert(obj, s, k)
            % insert in the queue vertex s with value k
            % if already exists change priority of s from k (old) to k
            
            pos = obj.find(s);
            if pos == -1
                s.k = k;
                obj.queue(end+1) = s;
                
                % TODO
                if s.state == Map.MAP_UNKNOWN || s.state == Map.MAP_EMPTY
                    s.state = Map.MAP_VISITED;
                end
            else
                s.k = k;
            end
            
        end
        
        function pos = find(obj, s)
            % find in the queue vertex s
            % if not exists return -1
            pos = -1;
            for i=1:length(obj.queue)
                if all(s==obj.queue(i))
                    pos=i;
                    return;
                end
            end
        end
        
        function res = isEmpty(obj)
            res = isempty(obj.queue);
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
            
            obj.queue(pos) = [];
        end
        
        function [minS, minV] = top(obj)
            % return a vertex with the smallest priority k
            % optional minV
            
            minV = [];
            minS = [];
            for elem=obj.queue
                if isempty(minV) || min2(elem.k, minV)
                    minV = elem.k;
                    minS = elem;
                end
            end
        end
        
        function minV = topKey(obj)
            % return the smallest priority k of all vertices
            % if empty return [+inf, +inf]
            minV = [];
            for elem=obj.queue
                if isempty(minV) || min2(elem.k, minV)
                    minV = elem.k;
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
    end
    
    % plot functions
    methods (Access = public)
        function plotPriorityQueue(obj)
            disp("Priority Queue:")
            for elem=obj.queue
                disp(strjoin(["    [", elem.x, ", ", elem.y, "] --> [", elem.k, "]"]))
            end
        end
    end
end


