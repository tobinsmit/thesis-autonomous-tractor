classdef controllerCompPID
    properties
        config
        path
        pathAngles
        pathCurvatures
        closestPointIdx = 1
    end
    methods
        
        function obj = setup(obj, config)
            obj.config = config;
            obj.path = config.path;
            x1 = obj.path(1:end-1,1);
            y1 = obj.path(1:end-1,2);
            x2 = obj.path(2:end,1);
            y2 = obj.path(2:end,2);
            dx = x2-x1;
            dy = y2-y1;
            obj.pathAngles = atan2(dy,dx);
            obj.pathAngles(end+1) = obj.pathAngles(end);
            pathStep = sqrt(mean(((x2-x1).^2+(y2-y1).^2)));
            obj.pathCurvatures = wrapToPi(diff(obj.pathAngles))/pathStep;
            obj.pathCurvatures(end+1) = obj.pathCurvatures(end);
        end
        
        function [obj, U, ctrlrOut] = loop(obj, X, t)
            %% Input
            x = X(1);
            y = X(2);
            theta = X(3);
            phi = X(4);

            %% Find tractor error
            [obj.closestPointIdx,minDis] = findClosestPoint(obj.path,X,obj.closestPointIdx);
            dx = obj.path(obj.closestPointIdx,1) - x;
            dy = obj.path(obj.closestPointIdx,2) - y;
            angleToPathGlobal = atan2(dy,dx);
            angleToPathRelative = wrapToPi(angleToPathGlobal - theta);
            pathHeading = obj.pathAngles(obj.closestPointIdx);
            pathCurvature = obj.pathCurvatures(obj.closestPointIdx);
            errHeading = wrapToPi(pathHeading - theta-pi);
            errCurvature = (0-pathCurvature)*obj.config.L_t;

            if angleToPathRelative > 0
                errDisplacement = - minDis;
            else
                errDisplacement = minDis;
            end

        %     deltaT = - 2 * errHeading + 100*errDisplacement;
%             deltaT = - 2 * errHeading + 1000*errDisplacement + errCurvature;
            deltaT = - 10*errHeading - 1*errDisplacement + 0*1*errCurvature;

            %% Implement
            theta_i = theta + phi;
            xh = x + obj.config.L_h * cos(theta);
            yh = y + obj.config.L_h * sin(theta);
            xi = xh - obj.config.L_i * cos(theta_i);
            yi = yh - obj.config.L_i * sin(theta_i);
            disSquaredI = (obj.path(:,1) - xi).^2 + (obj.path(:,2) - yi).^2;
            minDisSquaredI = min(disSquaredI);
            closestPointIdxI = find(disSquaredI == minDisSquaredI, 1, 'last');
            dxi = obj.path(closestPointIdxI,1) - x;
            dyi = obj.path(closestPointIdxI,2) - y;
            angleToPathGlobalI = atan2(dyi,dxi);
            angleToPathRelativeI = wrapToPi(angleToPathGlobalI - theta_i);
            pathHeadingI = obj.pathAngles(closestPointIdxI);
            pathCurvatureI = obj.pathCurvatures(closestPointIdxI);
            errHeadingI = wrapToPi(pathHeadingI - theta_i);
            errCurvatureI = (0-pathCurvatureI)*obj.config.L_i;

            if angleToPathRelativeI > 0
                % Vehicle is to the left of the path
                errDisplacementI = - sqrt(minDisSquaredI);
            else
                % Vehicle is to the right of the path
                errDisplacementI = sqrt(minDisSquaredI);
            end

            deltaI = 1*errHeadingI + 0*errDisplacementI + 0*errCurvatureI;
            maxDeltaI = inf;
            if deltaI > maxDeltaI, deltaI = maxDeltaI; end
            if deltaI < -maxDeltaI, deltaI = -maxDeltaI; end
            
            %% Output
            maxDeltaT = pi/4;
            if deltaT > maxDeltaT, deltaT = maxDeltaT; end
            if deltaT < -maxDeltaT, deltaT = -maxDeltaT; end
            U = [
                deltaT
                deltaI
            ];
            ctrlrOut.closestPointIdx = obj.closestPointIdx;
            ctrlrOut.pathHeading = pathHeading;
            ctrlrOut.errDisplacement = errDisplacement;
            ctrlrOut.errHeading = errHeading;
            ctrlrOut.errCurvature = errCurvature;
            ctrlrOut.s_t = 0;
        end

    end
end