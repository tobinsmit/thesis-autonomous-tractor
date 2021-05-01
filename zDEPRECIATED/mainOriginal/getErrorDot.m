function ed = getErrorDot(config, X, U);
    path = config.path;
    x = X(1);
    y = X(2);
    disSquared = (path(:,1) - x).^2 + (path(:,2) - y).^2;

end