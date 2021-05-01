function dX = plant(config, X, U)
    y = X(1);
    v = X(2);
    a = U(1);
    
    dy = v;
    dv = a;
    
    dX = [
        dy
        dv
    ];
end

