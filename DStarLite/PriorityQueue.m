classdef PriorityQueue
    properties
        queue; % [struct('vertex', s, 'key', k), struct('vertex', s, 'key', k)]
    end
    
    methods (Access = private)
        function str = builsStruct(s, k)
            str = struct('vertex', s, 'key', k);
        end
    end
    
    methods
        function obj = PriorityQueue(s, k)
            obj.queue = builsStruct(s, k);
        end
        
        function insert(obj, s, k)
            % insert vertex s in U with priority k
            obj.queue(end) = builsStruct(s, k)
        end
        
        function [] = remove()
            % remove a vertex s in U
            
        end
        
        function [] = top()
            % return a vertex with the smallest priority
            
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