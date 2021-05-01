classdef controllerCompPIDdesTrail
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
            x = X(1);
            y = X(2);
            theta = X(3);
            phi = X(4);
            
            v = obj.config.v;
            L_t = obj.config.L_t;
            L_h = obj.config.L_h;
            L_i = obj.config.L_i;

            %% Find tractor error
            [idx,minDis] = findClosestPoint(obj.path,X,obj.closestPointIdx);
            obj.closestPointIdx = idx;
            
            % Find signed distance to desired spot on path
            dx = obj.des.x(idx) - x;
            dy = obj.des.y(idx) - y;
            angleToPathGlobal = atan2(dy,dx);
            angleToPathRelative = wrapToPi(angleToPathGlobal - theta);
            if angleToPathRelative > 0
                e_offset = - minDis^4;
            else
                e_offset = minDis^4;
            end
            
            %
            
            % Find theta err
            theta_path = obj.des.theta(idx);
            e_theta = wrapToPi(theta_path - theta);
                        
            % Find delta_t err
            delta_t_path = obj.des.delta_t(idx);

            delta_t = delta_t_path - e_theta - e_offset;
            
            maxDeltaT = pi/4;
            if delta_t > maxDeltaT, delta_t = maxDeltaT; end
            if delta_t < -maxDeltaT, delta_t = -maxDeltaT; end

            %% Implement            
            % Find phi error
            phi_des = obj.des.phi(idx);
            e_phi = phi - phi_des;
                        
            % Find delta_i for chosen phi_dot
            c1 = v/L_i*sqrt(1+L_h^2/L_t^2*tan(delta_t)^2);
            c2 = atan(L_h/L_t*tan(delta_t)) + phi;
            c3 = v/L_t*tan(delta_t);
            
            phi_dot_des = -e_phi;
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