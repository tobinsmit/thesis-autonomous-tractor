function dX = plantAB(config,X,U)
    syms x y theta
    syms v delta
    syms L v
    
    X_sym = [x; y; theta];
    U_sym = [delta];
    
    x_dot = v*cos(theta);
    y_dot = v*sin(theta);
    theta_dot = L*v*tan(delta);

    dX_sym = [
        x_dot
        y_dot
        theta_dot
    ];

    A_sym = jacobian(dX_sym,X_sym);
    B_sym = jacobian(dX_sym,U_sym);
    
    A_sym_smallAng = applySmallAngleApproximations(A_sym);
    B_sym_smallAng = applySmallAngleApproximations(B_sym);
    
    dX_sym_rejoin = A_sym*X_sym + B_sym*U_sym;
    dX_sym_rejoin_smallAng = A_sym_smallAng*X_sym + B_sym_smallAng*U_sym;
    
    dX = zeros(3,size(X,2));
    for i = 1:size(X,2)
        x = X(1,i);
        y = X(2,i);
        theta = X(3,i);

        delta = U(1,i);
        
        L = config.L;
        v = config.v;

        dX(:,i) = subs(dX_sym_rejoin_smallAng);
    end

end

function M = applySmallAngleApproximations(M)
    M = mapSymType(M,'sin',@(x)children(x));
    M = mapSymType(M,'cos',@(x)1);
    M = mapSymType(M,'tan',@(x)children(x));
end