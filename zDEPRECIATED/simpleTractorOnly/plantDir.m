function dX = plantDir(config,X,U)
    x = X(1,:);
    y = X(2,:);
    theta = X(3,:);
    phi = X(4,:);

    delta = U(1,:);
    
    v = config.v;
    L = config.L;

    x_dot = v.*cos(theta);
    y_dot = v.*sin(theta);
    theta_dot = v.*tan(delta)/L;

    dX = [
        x_dot
        y_dot
        theta_dot
        0
    ];

end

