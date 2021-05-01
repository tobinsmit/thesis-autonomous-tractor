function U = controller(config,t,X)    
    U = [
        deg2rad(30)
        0 % delta_i
    ];

    U = [
        pi/4*cos(t)
        0 % delta_i
    ];

    U = [
        pi/4*cos(t)
        pi/4*cos(t)
    ];
end

