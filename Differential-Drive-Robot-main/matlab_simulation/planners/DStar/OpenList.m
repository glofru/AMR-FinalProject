classdef OpenList < handle
    properties (Access = private)
        queueS;
    end
    
    methods (Access = public)
        function obj = OpenList()
            obj.queueS = [];
        end
        
        function insert(obj, s)
            if isempty(obj.queueS)
                obj.queueS = s;
            else
                obj.queueS(end+1) = s;
            end
        end
        
        function pos = find(obj, s)
            % find in the queue vertex s
            % if not exists return -1
            pos = -1;
            for i=1:size(obj.queueS, 2)
                if s.eq(obj.queueS(i))
                    pos=i;
                    return;
                end
            end
        end
        
        function res = is_empty(obj)
            res = isempty(obj.queueS);
        end
        
        function h = has(obj, s)
            if obj.find(s) == -1
                h = false;
            else
                h = true;
            end
        end
        
        function remove(obj, s)
            % remove from the queue vertex s
            pos = obj.find(s);
            obj.queueS(pos) = [];
        end

        function k = get_kmin(obj)
            if isempty(obj.queueS)
                k = -1;
            else
                k = Inf;
                for e=obj.queueS
                    if e.k < k
                        k = e.k;
                    end
                end
            end
        end

        function [k, s] = min_state(obj)
            if isempty(obj.queueS)
                k = -1;
                s = State.empty;
            else
                k = Inf;
                for e=obj.queueS
                    if e.k < k
                        k = e.k;
                        s = e;
                    end
                end
            end
        end

        function p = print(obj)
            disp("OL")
            for i=obj.queueS
                disp(i.k)
            end
            disp("END")
        end
    end
end


