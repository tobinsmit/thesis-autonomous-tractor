classdef testClass
    properties
        x
        y
    end
    methods
        function obj = setup(obj,i,j)
            obj.x = i;
            obj.y = j;
        end
        function [obj,u] = loop(obj)
            obj.x = obj.x + 1;
            disp(obj.x)
            u = obj.x^2;
        end
    end
end