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
            % insert in the queue vertex s
            for i=1:size(obj.queueS, 2)
                if all(s==obj.queueS(:, i))
                    pos=i;
                    return;
                end
            end
        end
        
        function obj = remove(obj, s)
            % remove from the queue vertex s
            pos = obj.find(s);
            
            obj.queueS(:, pos) = [];
            obj.queueK(:, pos) = [];
        end
        
        function minS = top(obj)
            % return a vertex with the smallest priority k
            minV = [];
            minS = [];
            for i=1:size(obj.queueS, 2)
                if isempty(minV) || min2(obj.queueS(:, i), minV)
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
                if isempty(minV) || min2(obj.queueS(:, i), minV)
                    minV = obj.queueK(:, i);
                end
            end
        end
        
        function obj = pop(obj)
            % delete from the queue the vertex with the smallest priority k
            % and return it
            s = top(obj);
            obj = obj.remove(s);
        end
        
        function obj = update(obj, s, k)
            % change priority of s from k (old) to k
            pos = obj.find(s);
            obj.queueK(:, pos) = k; 
        end
    end
end