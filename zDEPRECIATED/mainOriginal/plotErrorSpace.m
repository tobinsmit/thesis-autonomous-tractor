clear
pathFile = 'paths/pathProvidedLap.mat';
path = load(pathFile).path;
config.path = path;

xmin = min(path(:,1));
xmax = max(path(:,1));
ymin = min(path(:,2));
ymax = max(path(:,2));

xmin = xmin - 0.2*(xmax-xmin);
xmax = xmax + 0.2*(xmax-xmin);
ymin = ymin - 0.2*(ymax-ymin);
ymax = ymax + 0.2*(ymax-ymin);
plot(path(:,1),path(:,2));
xlim([xmin xmax]);
ylim([ymin ymax]);

xs = linspace(xmin,xmax,100);
ys = linspace(ymin,ymax,100);

e = zeros(100,100);
idx = 1;
for xi = 1:100
    for yi = 1:100
        x = xs(xi);
        y = ys(yi);
        disSquared = (path(:,1) - x).^2 + (path(:,2) - y).^2;
        [minDisSquared, closestPointIdx] = min(disSquared);

%         [dis, idx] = findClosestPoint(config, [x;y], idx);
        e(xi,yi) = sqrt(minDisSquared);
    end
end

surf(xs,ys,e)