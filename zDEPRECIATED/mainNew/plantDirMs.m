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
    
    mt = 0.836333406488711;
    mp = 0.959793260515307;
    mi = 1.030616988384399;
    phi_dot = mt*delta_t + mp*phi + mi*delta_i;

    dX = [
        x_dot
        y_dot
        theta_t_dot
        phi_dot
    ];

end

