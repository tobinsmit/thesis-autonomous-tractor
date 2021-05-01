classdef controllerCompPIDdesTrail2
    properties
        config
        path
        des
        closestPointIdx = 1
        ei_phi = 0;
    end
    methods
        
        function obj = setup(obj, config)
            obj.config = config;
            obj.path = config.path;
            obj.des = findDesiredTrail2(config, config.path);
        end
        
        function r = canContinue(obj)
            r = obj.closestPointIdx < obj.config.pathLen-1;
        end
                
        function [obj, U, ctrlrOut] = loop(obj, X, t)
            %% Input
            xr = X(1);
            yr = X(2);
            theta = X(3);
            phi = X(4);
            
            v = obj.config.v;
            L_t = obj.config.L_t;
            L_h = obj.config.L_h;
            L_i = obj.config.L_i;

            %% Find tractor error
            [idx,minDis] = findClosestPoint(obj.path,X,obj.closestPointIdx);
            obj.closestPointIdx = idx;
            theta_p = obj.des.theta(idx);

            x_p = obj.des.x(idx);
            y_p = obj.des.y(idx);
            e_offset = (xr - x_p)*sin(theta_p) - (yr - y_p)*cos(theta_p);

            % Find theta_dot_des
            theta_dot_p = obj.des.theta_t_dot(idx);
            
            % Find theta err
            e_theta = wrapToPi(theta - theta_p);
                        
            theta_dot_des = theta_dot_p - e_theta - e_offset;
            delta_t = atan(L_t/v*theta_dot_des);
            
            maxDeltaT = pi/3;
            if delta_t > maxDeltaT, delta_t = maxDeltaT; end
            if delta_t < -maxDeltaT, delta_t = -maxDeltaT; end

            %% Implement            
            % Find phi error
            phi_p = obj.des.phi(idx);
            e_phi = phi - phi_p;
                        
            % Find delta_i for chosen phi_dot
            c1 = v/L_i*sqrt(1+L_h^2/L_t^2*tan(delta_t)^2);
            c2 = atan(L_h/L_t*tan(delta_t)) + phi;
            c3 = v/L_t*tan(delta_t);
            
            phi_dot_p = obj.des.phi_dot(idx);
            phi_dot_des = phi_dot_p - e_phi;
            delta_i = atan(-(phi_dot_des+c3)/c1/cos(c2)-tan(c2));

            maxDeltaI = pi/3;
            if delta_i > maxDeltaI, delta_i = maxDeltaI; end
            if delta_i < -maxDeltaI, delta_i = -maxDeltaI; end

            %% Output           
            
            U = [
                delta_t
                delta_i
            ];
            ctrlrOut.closestPointIdx = obj.closestPointIdx;
        end

    end
end