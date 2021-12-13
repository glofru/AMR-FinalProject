classdef StateCell
    properties
        %
        g;
        %
        rhs;
        % h
    end
    
    methods
        function obj = StateCell(g, rhs)
            arguments
                %
                g
                %
                rhs
            end
            obj.g = g;
            obj.rhs = rhs;
        end
        
        function [K1, K2] = CalcKey(obj)
            arguments
                %
                obj
            end

            K1 = min(obj.g, obj.rhs + 0); %+ h(start, s));
            K2 = min(obj.g, obj.rhs);
        end
    end
end