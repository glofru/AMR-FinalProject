classdef PriorityQueue
    properties
        queueS;
        queueK;
    end
    
    methods (Access = private)
        function str = builsStruct(s, k)
            str = struct('vertex', s, 'key', k);
        end
    end
    
    methods
        function obj = PriorityQueue()
            obj.queueS = [];
            obj.queueK = [];
        end
        
        function obj = insert(obj, s, k)
            obj.queueS(:, end+1) = s;
            obj.queueK(:, end+1) = k;
        end
        
        function pos = find(obj, s)
            for i=1:size(obj.queueS, 2)
                if all(s==obj.queueS(i))
                    pos=i;
                end
            end
        end
        
        function [] = remove(obj, s)
            % remove a vertex s in U
            pos = obj.find(s);
            
            obj.queueS(:, pos) = [];
            obj.queueK(:, pos) = [];
        end
        
        function res = min2(v1, v2)
            if (v1(1) < v2(1))
                res = true;
            elseif (v1(1) == v2(1))
                if (v1(2) < v2(2))
                    res = true;
                elseif (v1(2) == v2(2))
                    res = false;
                else % (v1(2) > v2(2))
                    res = false;
                end
            else % (v1(1) > v2(1))
                res = false;
            end
        end
        
        function minS = top(obj)
            % return a vertex with the smallest priority
            minV = [];
            minS = [];
            for i=1:size(obj.queueS, 2)
                if isempty(minV) || min2(obj.queueS(i), minV)
                    minV = obj.queueS(i);
                    minS = i;
                end
            end
        end
        
        function [] = topKey()
            % return the smallest priority of all vertices
            % if empty return [+inf, +inf]
            
        end
        
        function [] = pop()
            % delete the vertex with the smallest priority in U
            % and return it
            
        end
        
        function [] = update()
            % change priority of s in U to k
            
        end
    end
end