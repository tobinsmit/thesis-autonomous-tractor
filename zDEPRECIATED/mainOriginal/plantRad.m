function dX = plant(config,X,U)
    Lt = config.Lt;
    Lh = config.Lh;
    Li = config.Li;

    x = X(1);
    y = X(2);
    thetaT = X(3);
    phi = X(4);
    
    v = U(1);
    deltaT = U(2);
    deltaI = U(3);
    
    x_dot = v*cos(thetaT);
    y_dot = v*sin(thetaT);
    
    r1 = Lt / tan(deltaT);
    r2 = sqrt(r1^2+Lh^2);
    alpha1 = atan2(Lh, r1);
    alpha2 = pi/2 - alpha1 - phi;
    alpha3 = pi/2 - deltaI;
    alpha4 = pi - alpha2 - alpha3;
    r3 = Li * sin(alpha3) / sin(alpha4);
    thetaT_dot = v / r1;
    vh = thetaT_dot * r2;
    if isnan(vh)
        vh = v;
    end
    thetaI_dot = vh / r3;
    phi_dot = thetaI_dot - thetaT_dot;
    
%     rtr = Lt / tan(deltaT);
%     thetaT_dot = rad2deg(v / rtr);
%         
%     rth = sqrt(rtr^2 + Lh^2);
%     vh = v * rth / rtr;
%     alpha = atan(Lh/rtr);
%     rih = Li * sin(pi/2 - deltaI) / sin(alpha + phi + deltaI);
%     thetaI_dot = rad2deg(vh / rih);
%     phi_dot = thetaI_dot - thetaT_dot;
    
    dX = [
        x_dot
        y_dot
        thetaT_dot
        phi_dot
    ];
end

