classdef controllerSine
    methods(Static)
        function config = setup(config)
        end
        function [U, ctrlrOut] = loop(config,X,t,lastClosestPointIdx)
            delta = pi/6 * cos(t);

            U = [
                delta
                config.v
            ];
            ctrlrOut.closestPointIdx = lastClosestPointIdx;
        end
    end
end