function S = splinify(config, X, lastClosestPointIdx)
    %% Input
    x = X(1);
    y = X(2);
    thetaT = X(3) - 180;
    phi = X(4);

    %% Localise config variables
    path = config.path;
    horizon = config.horizon;
    inputOptions = config.inputOptions;
    lenX = config.lenX;
    lenU = config.lenU;

    %% Find spot on path
    [closestPointDis,closestPointIdx] = findClosestPoint(config,X,lastClosestPointIdx);

    
    horizon = min(horizon,length(config.path)-closestPointIdx); % Make sure horizon doesn't go past end of path
    
end