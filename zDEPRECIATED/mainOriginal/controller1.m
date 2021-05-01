function U = steer1(config, X, t)
    x = X(1);
    y = X(2);
    headingT = X(3);
    phi = X(4);
    
    path = config.path;
    dis = (path(:,1) - x).^2 + (path(:,2) - y).^2;
    closestPointIdx = find(dis == min(dis), 1, 'last');
    nextPoint = path(min(closestPointIdx + 5,length(config.path)),:);
    deltaPoint = atan2d(nextPoint(2) - y, nextPoint(1) - x);
    displacementAngle = deltaPoint - headingT;
    deltaT = - wrapTo180(displacementAngle);
    
    headingI =headingT+phi;
    xh = x + config.Lh * cosd(headingT);
    yh = y + config.Lh * sind(headingT);
    xi = xh - config.Li * cosd(headingI);
    yi = yh - config.Li * sind(headingI);

    deltaI = 0;

    v = 1;

    U = [
        v
        deltaT
        deltaI
        closestPointIdx
    ]';
end

   