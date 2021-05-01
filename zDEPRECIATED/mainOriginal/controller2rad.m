function [U, ctrlrOut] = controller2(config, X, t,lastClosestPointIdx)
    %% Input
    x = X(1);
    y = X(2);
    headingT = X(3);
    phi = X(4);
    
    %% Tractor
    path = config.path;
    
    % Avg = 0.41s
%     disSquared = (path(:,1) - x).^2 + (path(:,2) - y).^2;
%     [minDisSquared, closestPointIdx] = min(disSquared);

    % Avg = 0.41s
    [closestPointDisSquared,closestPointIdx] = findClosestPoint(config,X,lastClosestPointIdx);
    
    %%
    minDis = sqrt(closestPointDisSquared);
    dx = path(closestPointIdx,1) - x;
    dy = path(closestPointIdx,2) - y;
    angleToPathGlobal = atan2(dy,dx);
    angleToPathRelative = wrapToPi(angleToPathGlobal - headingT);
    pathHeading = config.pathAngles(closestPointIdx);
    pathCurvature = config.pathCurvatures(closestPointIdx);
    errHeading = wrapToPi(pathHeading - headingT);
    errCurvature = (0-pathCurvature)*config.Lt;
    
    if angleToPathRelative > 0
        % Vehicle is to the left of the path
        errDisplacement = - minDis;
    else
        % Vehicle is to the right of the path
        errDisplacement = minDis;
    end
    
%     deltaT = - 2 * errHeading + 100*errDisplacement;
    deltaT = - 2 * errHeading + 1000*errDisplacement + errCurvature;
    maxDeltaT = pi/4;
    if deltaT > maxDeltaT, deltaT = maxDeltaT; end
    if deltaT < -maxDeltaT, deltaT = -maxDeltaT; end
    
    %% Implement
    headingI = headingT+phi;
    xh = x + config.Lh * cos(headingT);
    yh = y + config.Lh * sin(headingT);
    xi = xh - config.Li * cos(headingI);
    yi = yh - config.Li * sin(headingI);
    disSquaredI = (path(:,1) - xi).^2 + (path(:,2) - yi).^2;
   
    minDisSquaredI = min(disSquaredI);
%     minDisI = sqrt(minDisSquaredI);
    closestPointIdxI = find(disSquaredI == minDisSquaredI, 1, 'last');
%     dxi = path(closestPointIdxI,1) - x;
%     dyi = path(closestPointIdxI,2) - y;
%     angleToPathGlobalI = atan2d(dyi,dxi);
%     angleToPathRelativeI = wrapToPi(angleToPathGlobalI - headingI);
    pathHeadingI = config.pathAngles(closestPointIdxI);
%     pathCurvatureI = config.pathCurvatures(closestPointIdxI);
    errHeadingI = wrapToPi(pathHeadingI - headingI);
%     errCurvatureI = (0-pathCurvatureI)*config.Li;
    
%     if angleToPathRelativeI > 0
%         % Vehicle is to the left of the path
%         errDisplacementI = - minDisI;
%     else
%         % Vehicle is to the right of the path
%         errDisplacementI = minDisI;
%     end
    
%     deltaT = - 2 * errHeading + 100*errDisplacement;
    deltaI = errHeadingI;
    maxDeltaI = inf;
    if deltaI > maxDeltaI, deltaI = maxDeltaI; end
    if deltaI < -maxDeltaI, deltaI = -maxDeltaI; end
   

    %% Output
    v = 1;
    U = [
        v
        deltaT
        deltaI
    ];
    ctrlrOut.closestPointIdx = closestPointIdx;
    ctrlrOut.pathHeading = pathHeading;
    ctrlrOut.errDisplacement = errDisplacement;
    ctrlrOut.errHeading = errHeading;
    ctrlrOut.errCurvature = errCurvature;
    ctrlrOut.xi = xi;
    ctrlrOut.yi = yi;
end

   