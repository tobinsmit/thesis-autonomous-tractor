classdef controllerSmc3spline
    properties
        % Constants
        config
        path
        des
        pathAngles
        pathIdxs
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
            idx = findClosestSplinePoint(obj.path,X);
            obj.closestPointIdx = idx;
            xs = spline(obj.pathIdxs, obj.xSs, idx);
            ys = spline(obj.pathIdxs, obj.ySs, idx);
            xo = spline(obj.pathIdxs, obj.xOs, idx);
            yo = spline(obj.pathIdxs, obj.yOs, idx);

            % Tractor SMC
            gamma = wrapToPi(spline(obj.pathIdxs, obj.pathAngles, idx) - theta + pi);
            e_t = xs*(x - xo)+ ys*(y - yo);
            ed_t = -v*sin(gamma);
            s_t = e_t + ed_t;
%             if abs(s_t) > 0.1
            if abs(s_t) > inf
                % Reaching
                delta_t = atan(-L_t*(2*ed_t + e_t)/(v^2*cos(gamma)));
            else
                % Sliding
                delta_t_sliding_margin = deg2rad(10);
                if -v*sin(gamma) + xs*(x - xo) + ys*(y - yo) > 0
                    delta_t1 = -pi/4;
                    delta_t2 = atan(L_t/v*tan(gamma)) - delta_t_sliding_margin;
                else
                    delta_t1 = pi/4;
                    delta_t2 = atan(L_t/v*tan(gamma)) + delta_t_sliding_margin;
                end
                delta_t = delta_t2;
            end

            if delta_t > obj.maxDelta, delta_t = obj.maxDelta; end
            if delta_t < -obj.maxDelta, delta_t = -obj.maxDelta; end

            % Implement SMC
            % Find phi error
            phi_des = spline(obj.pathIdxs, obj.des.phi, idx);
            phi_dot_des = spline(obj.pathIdxs, obj.des.phi_dot, idx);
            phi_dot_dot_des = spline(obj.pathIdxs, obj.des.phi_dot_dot, idx);
            
            k1 = v/L_i*sqrt(1+L_h^2/L_t^2*tan(delta_t)^2);
            k2 = atan(L_h/L_t*tan(delta_t)) + phi;
            k3 = v/L_t*tan(delta_t);

            s = sin(k2);
            c = cos(k2);
            
            A = - k1^2*s*c;
            B = k1^2*c^2 - k1^2*s^2 - k1*k3*s - 2*k1*c;
            C = k1^2*s*c + k1*k3*c - 2*k1*s - 2*k3 - phi_dot_dot_des - 2*phi_dot_des - phi_des + phi;
            
            disc = (B^2 - 4*A*C)^(1/2);
            roots = [(-B + disc)/(2*A), (-B - disc)/(2*A)];
            ds = atan(roots);
            [~,di] = min(abs(ds));
            delta_i = ds(di);
            
            if delta_i > obj.maxDelta, delta_i = obj.maxDelta; end
            if delta_i < -obj.maxDelta, delta_i = -obj.maxDelta; end

            % Output
            U = [
                delta_t
                delta_i
            ];
            obj.lastDelta = delta_t;
            ctrlrOut.closestPointIdx = round(idx);
            ctrlrOut.s_t = s_t;
            ctrlrOut.gamma = gamma;
        end

    end
end