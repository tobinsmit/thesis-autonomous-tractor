function [idxs, Ps,dissSquared] = findClosestSplinePointMat(path, X)
    n = size(X,2);
    idxs = zeros(1,n);
    Ps = zeros(2,n);
    dissSquared = zeros(1,n);
    for j = 1:n
        x = X(1,j);
        y = X(2,j);
        
        % Create spline
        pathIdxs = 1:length(path);
        spln = spline(pathIdxs,path');
        
        % Find closest integer index
        disSquared = (path(:,1) - x).^2 + (path(:,2) - y).^2;
        [~, idx] = min(disSquared);
        
        % Find closest float index around integer index
        f = @(idx) distanceFromSplineIdx(idx,x,y,spln);
        [idx, disSquared] = fminsearch(f,idx);
        
        idx = max(1,min(length(path),idx));
        P = ppval(spln,idx);
        
        idxs(:,j) = idx;
        Ps(:,j) = P;
        dissSquared(:,j) = disSquared;
    end
end

function disSquared = distanceFromSplineIdx(idx,x,y,spln)
    Xpath = ppval(spln,idx);
    disSquared = (Xpath(1) - x)^2 + (Xpath(2) - y)^2;
end
