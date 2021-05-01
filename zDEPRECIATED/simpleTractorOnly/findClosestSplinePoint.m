function [i,P] = findClosestSplinePoint(path, X)
    x = X(1);
    y = X(2);
    u = 1:length(path);
    sp = spline(u,path');
    i = findClosestUToPoint(x,y,sp,path);
    P = ppval(sp,i);
end

function error = distanceFromS(u,x,y,sp)
    Xpath = ppval(sp,u);
    error = (Xpath(1) - x)^2 + (Xpath(2) - y)^2;
end

function u = findClosestUToPoint(x,y,sp,path)
    disSquared = (path(:,1) - x).^2 + (path(:,2) - y).^2;
    [~, closestPointIdx] = min(disSquared);

    f = @(s) distanceFromS(s,x,y,sp);
%     u = fminsearch(f,closestPointIdx);
    u = fminbnd(f,1,length(path));
end