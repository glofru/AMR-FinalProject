classdef OpenList < handle
    properties (Access = private)
        actualList;
    end

    methods (Access = public)
        function obj = OpenList(initialState)
            obj.actualList = initialState;
        end

        function insert(obj, state, h_new)
            if state.tag == StateTag.NEW
                state.k = h_new;
            elseif state.tag == StateTag.OPEN
                state.k = min(state.k, h_new);
            elseif state.tag == StateTag.CLOSED
                state.k = min(state.h, h_new);
            end
            state.h = h_new;
            state.tag = StateTag.OPEN;

            if isempty(obj.actualList)
                obj.actualList = state;
            else
                obj.actualList(end+1) = state;
            end
        end

        function remove(obj, state)
            state.tag = StateTag.CLOSED;
            pos = obj.findPosition(state);
            if pos ~= -1
                obj.actualList(pos) = [];
            end
        end

        function pos = findPosition(obj, s)
            % find in the queue vertex s
            % if not exists return -1
            pos = -1;
            for i=1:size(obj.actualList, 2)
                if s.eq(obj.actualList(i))
                    pos = i;
                    return;
                end
            end
        end

        function k = get_kmin(obj)
            if isempty(obj.actualList)
                k = -1;
            else
                k = Inf;
                for e=obj.actualList
                    if e.k < k
                        k = e.k;
                    end
                end
            end
        end

        function s = min_state(obj)
            s = DState.empty(1, 0);
            if ~isempty(obj.actualList)
                k = Inf;
                for e=obj.actualList
                    if e.k < k
                        k = e.k;
                        s = e;
                    end
                end
            end
        end

        function e = isEmpty(obj)
            e = isempty(obj.actualList);
        end
    end
end