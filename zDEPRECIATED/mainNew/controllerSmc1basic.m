classdef controllerSmcComp
    properties
        % Constants
        config
        path
        des
        pathAngles
        m1s
        m2s
        xSs
        ySs
        xOs
        yOs

        maxDelta = deg2rad(60);

        % State Varibales
        lastDelta = 0
        
        % Other Variables
        closestPointIdx = 1
        
    end
    methods
        function obj = setup(obj, config)
            obj.config = config;
            obj.path = config.path;
            obj.des = config.des;
            
            % Calculate mean dt between path points
            x1 = obj.path(1:end-1,1);
            y1 = obj.path(1:end-1,2);
            x2 = obj.path(2:end,1);
            y2 = obj.path(2:end,2);
            pathStep = sqrt(mean((x2-x1).^2+(y2-y1).^2));
            pathDt = abs(pathStep./obj.config.v);

            % Calculate path angles with points either side
            x1 = obj.path([1 1:end-1],1);
            y1 = obj.path([1 1:end-1],2);
            x2 = obj.path([2:end end],1);
            y2 = obj.path([2:end end],2);
            obj.pathAngles = atan2(y2-y1,x2-x1);
            
            
            obj.m1s = (x2-x1)./pathDt;
            obj.m2s = (y2-y1)./pathDt;
            s = sqrt(obj.m1s.^2 + obj.m2s.^2);
            obj.xSs = obj.m2s./s;
            obj.ySs = -obj.m1s./s;
            obj.xOs = obj.path(:,1);
            obj.yOs = obj.path(:,2);
        end
        
        function r = canContinue(obj)
            r = obj.closestPointIdx < obj.config.pathLen-1;
        end
                
        function [obj, U, ctrlrOut] = loop(obj, X, t)
            % Input
            x = X(1);
            y = X(2);
            theta = X(3);
            phi = X(4);
            
            L_t = obj.config.L_t;
            L_h = obj.config.L_h;
            L_i = obj.config.L_i;
            v = obj.config.v;

            % Get closest path data
            idx = findClosestPoint(obj.path,X,obj.closestPointIdx);
            obj.closestPointIdx = idx;
            xs = obj.xSs(idx);
            ys = obj.ySs(idx);
            xo = obj.xOs(idx);
            yo = obj.yOs(idx);

            % Tractor SMC
            gamma = obj.pathAngles(idx) - theta;
            e_t = xs*(x - xo)+ ys*(y - yo);
            ed_t = v*sin(gamma);
            s_t = e_t + ed_t;
            if abs(s_t) > 0.1
                % Reaching
                delta_t = atan(t*(2*ed_t + e_t)/(v^2*cos(gamma)));
            else
                % Sliding
                if v*sin(gamma) + xs*(x - xo) + ys*(y - yo) > 0
                    delta_t = -pi/4;
                else
                    delta_t = pi/4;
                end
            end
            
            % Implement SMC
            % Find phi error
            phi_des = obj.des.phi(idx);
            e_phi = phi - phi_des;
                        
            % Find delta_i for chosen phi_dot
            c1 = v/L_i*sqrt(1+L_h^2/L_t^2*tan(delta_t)^2);
            c2 = atan(L_h/L_t*tan(delta_t)) + phi;
            c3 = v/L_t*tan(delta_t);
            
            phi_dot_des = -e_phi;
            delta_i = atan(-(phi_dot_des+c3)/c1/cos(c2)-tan(c2));
            
            % Output
            if delta_t > obj.maxDelta, delta_t = obj.maxDelta; end
            if delta_t < -obj.maxDelta, delta_t = -obj.maxDelta; end
            U = [
                delta_t
                delta_i
            ];
            obj.lastDelta = delta_t;
            ctrlrOut.closestPointIdx = idx;
            ctrlrOut.s_t = s_t;
        end

    end
end