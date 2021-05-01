function [U, ctrlrOut] = controllerSMC(config, X, t)
    e = sqrt(dx + dy);
    s = ed - e;
    

    ctrlrOut = {};
    U = [
        v
        deltaT
        deltaI
        minDisIdx
    ]';    
end