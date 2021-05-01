classdef controllerSmc
    properties
        % Constants
        config
        v
        L_t
        path
        pathAngles
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
            obj.L_t = config.L_t;
            
            % Calculate mean dt between path points
            x1 = obj.path(1:end-1,1);
            y1 = obj.path(1:end-1,2);
            x2 = obj.path(2:end,1);
            y2 = obj.path(2:end,2);
            pathStep = sqrt(mean((x2-x1).^2+(y2-y1).^2));
            pathDt = abs(pathStep./obj.v);

            % Calculate path angles with points either side
            x1 = obj.path([1 1:end-1],1);
            y1 = obj.path([1 1:end-1],2);
            x2 = obj.path([2:end end],1);
            y2 = obj.path([2:end end],2);
            obj.pathAngles = atan2(y2-y1,x2-x1);
            
            
            obj.m1s = zeros(length(obj.pathAngles),1);
            obj.m2s = zeros(length(obj.pathAngles),1);
            obj.m1s = (x2-x1)./pathDt;
            obj.m2s = (y2-y1)./pathDt;
            s = sqrt(obj.m1s.^2 + obj.m2s.^2);
            obj.xSs = obj.m2s./s;
            obj.ySs = -obj.m1s./s;
            obj.xOs = obj.path(:,1);
            obj.yOs = obj.path(:,2);
        end
                
        function [obj, U, ctrlrOut] = loop(obj, X, t)
            %% Input
            x = X(1);
            y = X(2);
            theta = X(3);
            phi = X(4);
            
            %% Get closest path data
            obj.closestPointIdx = findClosestPoint2(obj.path,X,obj.closestPointIdx);
            xs = obj.xSs(obj.closestPointIdx);
            ys = obj.ySs(obj.closestPointIdx);
            xo = obj.xOs(obj.closestPointIdx);
            yo = obj.yOs(obj.closestPointIdx);


            %% Reaching
            gamma = obj.pathAngles(obj.closestPointIdx) - theta;
            delta_t = atan(obj.L_t*(2*obj.v*sin(gamma) + xs*(x - xo)+ ys*(y - yo))/(obj.v^2*cos(gamma)));
            
            %% Sliding
%             if obj.v*sin(gamma) + xs*(x - xo) + ys*(y - yo) > 0
%                 thisDelta = -pi/4;
%             else
%                 thisDelta = pi/4;
%             end
%             
%             c = 0.5;
%             delta = obj.lastDelta*(1-c) + thisDelta*(c);


            
            %% Implement
%             k1 = 0.3012;
%             k2 = -0.9639;
%             k3 = -1.0298;
%             delta_i = -(10000*(1+2*k2+k2^2)*phi+50*(2*k1+k2*k1)*delta_t)/(-2*k3-k2*k3);
            delta_i = -5*phi - 5*delta_t;
            
            %% Output
            maxDelta = pi/4;
            if delta_t > maxDelta, delta_t = maxDelta; end
            if delta_t < -maxDelta, delta_t = -maxDelta; end
            if delta_i > maxDelta, delta_i = maxDelta; end
            if delta_i < -maxDelta, delta_i = -maxDelta; end
            obj.lastDelta = delta_t;
            ctrlrOut.closestPointIdx = obj.closestPointIdx;
            U = [
                delta_t
                delta_i;
            ];
        end

    end
end