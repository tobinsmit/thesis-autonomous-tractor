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
    
    x_dot = v*cosd(thetaT);
    y_dot = v*sind(thetaT);
    
    r1 = Lt / tand(deltaT);
    r2 = sqrt(r1^2+Lh^2);
    alpha1 = atan2d(Lh, r1);
    alpha2 = 90 - alpha1 - phi;
    alpha3 = 90 - deltaI;
    alpha4 = 180 - alpha2 - alpha3;
    r3 = Li * sind(alpha3) / sind(alpha4);
    thetaT_dot = rad2deg(v / r1);
    vh = deg2rad(thetaT_dot) * r2;
    if isnan(vh)
        vh = v;
    end
    thetaI_dot = rad2deg(vh / r3);
    phi_dot = thetaI_dot - thetaT_dot;
    
%     rtr = Lt / tand(deltaT);
%     thetaT_dot = rad2deg(v / rtr);
%         
%     rth = sqrt(rtr^2 + Lh^2);
%     vh = v * rth / rtr;
%     alpha = atand(Lh/rtr);
%     rih = Li * sind(90 - deltaI) / sind(alpha + phi + deltaI);
%     thetaI_dot = rad2deg(vh / rih);
%     phi_dot = thetaI_dot - thetaT_dot;
    
    dX = [
        x_dot
        y_dot
        thetaT_dot
        phi_dot
    ];
end

