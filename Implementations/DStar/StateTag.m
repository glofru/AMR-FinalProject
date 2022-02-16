classdef StateTag
    enumeration
        NEW
        OPEN
        CLOSED
    end

    methods
        function c = getColor(obj)
            switch obj
                case StateTag.NEW
                    c = [0, 0.4, 0.4];
                case StateTag.OPEN
                    c = [0, 1, 0];
                case StateTag.CLOSED
                    c = [0, 0, 1];
            end
        end
    end
end