function t = findDesiredTrail3(config, path)
    x = path(:,1);
    y = path(:,2);
    
    L_t = config.L_t;
    L_h = config.L_h;
    L_i = config.L_i;
    v = config.v;
    
    t = table(x,y);

    subIdxs = [1.5:1:length(x)]';
    fullIdxs = [1:length(x)]';

%     % Calc theta
%     x1 = x(1:end-1);
%     y1 = y(1:end-1);
%     x2 = x(2:end);
%     y2 = y(2:end);
%     subTheta = atan2(y1-y2,x1-x2);
%     
%     % Make continuous
%     subTheta = wrapToPi(subTheta(1)) + [0; cumsum(wrapToPi(diff(subTheta)))];
%     % Extrapolate
%     t.theta = spline(subIdxs,subTheta,fullIdxs);

    dx = gradient(x);
    dy = gradient(y);
    subTheta = atan2(-dy,-dx);
    
    % Make continuous
    subTheta = wrapToPi(subTheta(1)) + [0; cumsum(wrapToPi(diff(subTheta)))];
    % Extrapolate
    t.theta = subTheta;
    

    % Calc theta_t_dot
    pathStep = sqrt(mean(dx).^2 + mean(dy).^2);
    pathDt = abs(pathStep/config.v);
    t.theta_t_dot = gradient(t.theta) / pathDt;
    
    delta_i = 0;
    t.delta_t = atan(L_t*t.theta_t_dot/v);

    % Find phi
    t.phi = zeros(size(t.theta));
    for i = length(path)-1:-1:1
        lastPhiF = t.phi(i+1);
        delta_t = t.delta_t(i);
        v = 1;
%         f = @(t,phi)- (v*tan(delta_t))/L_t - (v*sin(delta_i + phi + atan((L_h*tan(delta_t))/L_t))*tan(delta_t)*(L_t^2/tan(delta_t)^2 + L_h^2)^(1/2))/(L_i*L_t*cos(delta_i));
%         f = @(t,phi) (v*sin(delta_i + phi + pi + atan((L_h*tan(delta_t))/L_t)).*((L_h^2*tan(delta_t).^2)/L_t^2 + 1).^(1/2))/(L_i*cos(delta_i)) - (v*tan(delta_t))/L_t;
        f = @(t,phi) getPhiDot(config,phi,delta_t,delta_i,v);
        [~,ps] = ode45(f,[0 pathDt],lastPhiF);
        t.phi(i) = ps(end);
    end
    
    t.phi_dot = spline(subIdxs,diff(t.phi)/pathDt,fullIdxs);  
    t.phi_dot_dot = spline(subIdxs,diff(t.phi_dot)/pathDt,fullIdxs);  

    % Find x,y of implement (xi,yi)
    t.xf = x + config.L_t*cos(t.theta);
    t.yf = y + config.L_t*sin(t.theta);
    t.xh = x - config.L_h*cos(t.theta);
    t.yh = y - config.L_h*sin(t.theta);
    t.xi = t.xh - config.L_i*cos(t.theta + t.phi);
    t.yi = t.yh - config.L_i*sin(t.theta + t.phi);

    % Find delta_i for chosen phi_dot
    v = -1;
    c1 = v/L_i*sqrt(1+L_h^2/L_t^2*tan(t.delta_t).^2);
    c2 = atan(L_h*tan(t.delta_t)/L_t) + t.phi;
    c3 = v*tan(t.delta_t)/L_t;

    t.delta_i = atan(-(t.phi_dot+c3)./c1./cos(c2) - tan(c2));
    
    t.radialAccel = v.*t.theta_t_dot;
%     t.delta_i = zeros(size(t.delta_t));
    
%     figure(1), clf, hold on
%     plot(t.delta_i)
%     plot(t.phi_dot)
%     phi_dot_from_delta_i = c1.*sin(c2+t.delta_i)./cos(delta_i)-c3;
%     phi_dot_from_phi_dot = c1.*sin(c2+(atan((t.phi_dot+c3)./c1./cos(c2) - tan(c2))))./cos(atan((t.phi_dot+c3)./c1./cos(c2) - tan(c2)))-c3;
%     plot(phi_dot_from_delta_i,'--')
%     plot(phi_dot_from_phi_dot,'--')
%     legend('delta i','phi dot','phi dot from delta i','phi dot from phi dot')
end

function phi_dot = getPhiDot(config,phi,delta_t,delta_i,v)
    L_t = config.L_t;
    L_h = config.L_h;
    L_i = config.L_i;

    r1 = L_t / tan(delta_t);
    r2 = sqrt(r1^2+L_h^2);
    r3 = L_i * cos(delta_i) / sin(atan2(L_h,r1) + phi + delta_i);
    theta_t_dot = (v / r1);
    vh = (theta_t_dot) * r2;
    if isnan(vh)
        vh = v;
    end
    theta_i_dot = -(vh / r3);
    phi_dot = theta_i_dot - theta_t_dot;
end
