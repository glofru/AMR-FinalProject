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
                    c = [0, 0, 0];
                case StateTag.OPEN
                    c = [0, 255, 0];
                case StateTag.CLOSED
                    c = [0, 0, 255];
            end
        end
    end
end