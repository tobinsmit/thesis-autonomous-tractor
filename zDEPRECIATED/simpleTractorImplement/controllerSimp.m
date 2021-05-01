classdef controllerSimp
    properties
        % Constants
        config
        v
        L_t
        path

        % Other Variables
        closestPointIdx = 1
        
    end
    methods
        function obj = setup(obj, config)
            obj.config = config;
            obj.path = config.path;
            obj.v = config.v;
            obj.L_t = config.L_t;
            
        end
                
        function [obj, U, ctrlrOut] = loop(obj, X, t)
            %% Input
            x = X(1);
            y = X(2);
            theta = X(3);
            phi = X(4);
            
            %% Get closest path data
            obj.closestPointIdx = findClosestPoint2(obj.path,X,obj.closestPointIdx);

            delta_t = pi/6;
            delta_i = pi/6;
            delta_i = 0;

            ctrlrOut.closestPointIdx = obj.closestPointIdx;
            U = [
                delta_t
                delta_i
            ];
        end

    end
end