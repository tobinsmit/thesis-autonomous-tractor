clear
pathFile = '../paths/path7';
path = load(pathFile).path;
config.path = path;
s = 1:length(path);
pp = spline(s,path');

P1 = [-1,1];
si = findClosestSToPoint(P1,pp,path);
P2 = ppval(pp,si);

figure(1), clf, hold on, axis equal
plot(path(:,1),path(:,2),'.')
plot(path(ceil(si),1),path(ceil(si),2),'+')
plot(path(floor(si),1),path(floor(si),2),'+')
plot(P1(1),P1(2),'go')
plot(P2(1),P2(2),'ro')

function error = distanceFromS(s,X,pp)
    Xpath = ppval(pp,s);
    error = (Xpath(1) - X(1))^2 + (Xpath(2) - X(2))^2;
end

function s = findClosestSToPoint(X,pp,path)
    x = X(1);
    y = X(2);
    disSquared = (path(:,1) - x).^2 + (path(:,2) - y).^2;
    [~, closestPointIdx] = min(disSquared);

    f = @(s) distanceFromS(s,X,pp);
    s = fminsearch(f,closestPointIdx);
    s = fminbnd(f,1,length(path));
end