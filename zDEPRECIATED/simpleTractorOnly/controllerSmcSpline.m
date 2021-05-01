classdef controllerSmcSpline
    properties
        % Constants
        config
        v
        L
        path
        pathAngles
        pathIdxs
        m1s
        m2s
        xSs
        ySs
        xOs
        yOs
        
        % State Varibales
        lastDelta = 0
        
        % Other Variables
        closestPointIdx = 1
        
    end
    methods
        function obj = setup(obj, config)
            obj.config = config;
            obj.path = config.path;
            obj.v = config.v;
            obj.L = config.L;
            
            % Calculate mean dt between path points
            x1 = obj.path(1:end-1,1);
            y1 = obj.path(1:end-1,2);
            x2 = obj.path(2:end,1);
            y2 = obj.path(2:end,2);
            pathStep = sqrt(mean((x2-x1).^2+(y2-y1).^2));
            pathDt = abs(pathStep./obj.v);
            obj.pathIdxs = 1:length(obj.path);

            % Calculate path angles with points either side
            x1 = obj.path([1 1:end-1],1);
            y1 = obj.path([1 1:end-1],2);
            x2 = obj.path([2:end end],1);
            y2 = obj.path([2:end end],2);
            obj.pathAngles = atan2(y2-y1,x2-x1);
            % Make pathAngles continuous
            obj.pathAngles = cumsum([obj.pathAngles(1); wrapToPi(diff(obj.pathAngles))]);
            
            
            obj.m1s = (x2-x1)./pathDt;
            obj.m2s = (y2-y1)./pathDt;
            s = sqrt(obj.m1s.^2 + obj.m2s.^2);
            obj.xSs = obj.m2s./s;
            obj.ySs = -obj.m1s./s;
            obj.xOs = obj.path(:,1);
            obj.yOs = obj.path(:,2);
            
            % Plot tangential quivers
%             quiver(obj.xOs,obj.yOs,obj.m1s,obj.m2s,0.05)
        end
                
        function [obj, U, ctrlrOut] = loop(obj, X, t)
            %% Input
            x = X(1);
            y = X(2);
            theta = X(3);
            
            %% Get closest path data
            [obj.closestPointIdx,~] = findClosestSplinePoint(obj.path,X);
            xs = spline(obj.pathIdxs, obj.xSs, obj.closestPointIdx);
            ys = spline(obj.pathIdxs, obj.ySs, obj.closestPointIdx);
            xo = spline(obj.pathIdxs, obj.xOs, obj.closestPointIdx);
            yo = spline(obj.pathIdxs, obj.yOs, obj.closestPointIdx);


            %% Reaching
            gamma = spline(obj.pathIdxs, obj.pathAngles, obj.closestPointIdx) - theta;
            delta = atan(obj.L*(2*obj.v*sin(gamma) + xs*(x - xo)+ ys*(y - yo))/(obj.v^2*cos(gamma)));
            
            %% Sliding
%             if obj.v*sin(gamma) + xs*(x - xo) + ys*(y - yo) > 0
%                 thisDelta = -pi/4;
%             else
%                 thisDelta = pi/4;
%             end
%             
%             c = 0.5;
%             delta = obj.lastDelta*(1-c) + thisDelta*(c);

            %% Output
            maxDelta = pi/4;
            if delta > maxDelta, delta = maxDelta; end
            if delta < -maxDelta, delta = -maxDelta; end
            U = [
                delta
                0
            ];
            obj.lastDelta = delta;
            ctrlrOut.closestPointIdx = round(obj.closestPointIdx);
        end

    end
end