function [U, ctrlrOut] = steerSine(config, X, t, closestPointIdx)
    x = X(1);
    y = X(2);
    thetaT = X(3);
    phi = X(4);
    
    path = config.path;
    disSquared = (path(:,1) - x).^2 + (path(:,2) - y).^2;
    minDisSquared = min(disSquared);
    closestPointIdx = find(disSquared == minDisSquared);

    v = -1;
    deltaT = pi/12*cos(t*1.5);
    deltaI = 0;

    deltaT = pi/6;
    deltaI = pi/6;
    
    deltaT = 30;
    deltaI = 0;

    U = [
        v
        deltaT
        deltaI
    ];

    ctrlrOut.closestPointIdx = closestPointIdx;
end

   