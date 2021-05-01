function [idx,P,disSquared] = findClosestSplinePoint(path, X)
    x = X(1);
    y = X(2);
    
    % Create spline
    idxIntegers = 1:length(path);
    spln = spline(idxIntegers,path');
    
    % Find closest integer index
    disSquared = (path(:,1) - x).^2 + (path(:,2) - y).^2;
    [~, idx] = min(disSquared);
    
    % Find closest float index around integer index
    f = @(idx) distanceFromSplineIdx(idx,x,y,spln);
    [idx, disSquared] = fminsearch(f,idx);
    
    idx = max(1,min(length(path),idx));
    P = ppval(spln,idx);
end

function disSquared = distanceFromSplineIdx(idx,x,y,spln)
    Xpath = ppval(spln,idx);
    disSquared = (Xpath(1) - x)^2 + (Xpath(2) - y)^2;
end
