function dX = plantDir(config,X,U)
    x = X(1,:);
    y = X(2,:);
    theta = X(3,:);
    phi = X(4,:);

    delta_t = U(1,:);
    delta_i = U(2,:);
    
    v = config.v;
    L_t = config.L_t;
    L_h = config.L_h;
    L_i = config.L_i;

    x_dot = v.*cos(theta);
    y_dot = v.*sin(theta);
    theta_t_dot = v.*tan(delta_t)/L_t;
        
    r1 = L_t / tan(delta_t);
    r2 = sqrt(r1^2+L_h^2);
    r3 = L_i * cos(delta_i) / sin(atan2(L_h,r1) + phi + delta_i);
    theta_t_dot = v / r1;
    vh = (theta_t_dot) * r2;
    if isnan(vh)
        vh = v;
    end
    theta_i_dot = -vh / r3;
    phi_dot = theta_i_dot - theta_t_dot;

    dX = [
        x_dot
        y_dot
        theta_t_dot
        phi_dot
    ];

end

