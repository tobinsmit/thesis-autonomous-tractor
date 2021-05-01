function dX = plantWithAB(config,Xi,Ui)
    syms L_i L_t L_h
    assume(L_i,'positive')
    assume(L_t,'positive')
    assume(L_h,'positive')
    syms x y theta_t phi
    assume(x,'real')
    assume(y,'real')
    assume(theta_t,'real')
    assume(phi,'real')
    syms v delta_t delta_i
    assume(v,'real')
    assume(delta_t,'real')
    assume(delta_i,'real')


    dims = [L_t L_h L_i]';
    X = [x y theta_t phi]';
    U = [v delta_t delta_i]';
    % X1 = [x, y, theta_t]';
    % U1 = [v, delta_t]';
    % X2 = [x, y, theta_t, phi, delta_t]';
    % U2 = [delta_i]';

    x_dot = v*cos(theta_t);
    y_dot = v*sin(theta_t);

    r1 = (L_t / tan(delta_t));
    r2 = sqrt(r1^2+L_h^2);
    % alpha1 = atan2(L_h, r1)
    alpha1 = atan(L_h/r1);
    alpha4 = alpha1 + phi + delta_i;
    r3 = L_i * cos(delta_i) / sin(alpha4);
    theta_t_dot = v / r1;
    % vh = (theta_t_dot) * r2

    vh = v*sqrt(1+L_h^2*tan(delta_t)^2/L_t^2);

    % vh = simplify(vh,'IgnoreAnalyticConstraints',true)
    theta_i_dot = vh / r3;
    phi_dot = theta_i_dot - theta_t_dot;

    dX = [
        x_dot
        y_dot
        theta_t_dot
        phi_dot
    ];

    % dX1 = [
    %     x_dot
    %     y_dot
    %     theta_t_dot
    % ];
    % 
    % dX2 = [
    %     x_dot
    %     y_dot
    %     theta_t_dot
    %     phi_dot
    %     0
    % ];

    A = jacobian(dX,X);
    B = jacobian(dX,U);
    % A1 = jacobian(dX1,X1);
    % B1 = jacobian(dX1,U1);
    % A2 = jacobian(dX2,X2);
    % B2 = jacobian(dX2,U2);

    A = simplify(A,'IgnoreAnalyticConstraints',true);
    B = simplify(B,'IgnoreAnalyticConstraints',true);
    % A1 = simplify(A1,'IgnoreAnalyticConstraints',true);
    % B1 = simplify(B1,'IgnoreAnalyticConstraints',true);
    % A2 = simplify(A2,'IgnoreAnalyticConstraints',true);
    % B2 = simplify(B2,'IgnoreAnalyticConstraints',true);

    L_i = config.dims(1);
    L_t = config.dims(1);
    L_h = config.dims(1);

    x = Xi(1);
    y = Xi(2);
    theta_t = Xi(3);
    phi = Xi(4);
    
    v = Ui(1);
    delta_t = Ui(2);
    delta_i = Ui(3);
    
    A = subs(A);
    B = subs(B);

    dX = A*Xi + B*Ui;
end

function M = applySmallAngleApproximations(M)
    M = mapSymType(M,'sin',@(x)children(x));
    M = mapSymType(M,'cos',@(x)1);
    M = mapSymType(M,'tan',@(x)children(x));
end
