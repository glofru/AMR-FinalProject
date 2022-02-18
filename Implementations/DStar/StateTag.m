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
                    c = [0.2, 0.8, 0.2];
                case StateTag.CLOSED
                    c = [0.5, 0, 0.5];
            end
        end
    end
end