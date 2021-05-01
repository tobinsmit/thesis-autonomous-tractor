t = tic();
pathFile = 'paths/path7';
p = load(pathFile).path;
% config.path = p;
for i = 1:100000
    out = tester(p);
end
toc(t)

function out = tester(path)
%     path = config.path;
    x1 = path(1:end-1,1);
    y1 = path(1:end-1,2);
    x2 = path(2:end,1);
    y2 = path(2:end,2);
    dx = x2-x1;
    dy = y2-y1;
    pathAngles = atan2d(dy,dx);
    pathAngles(end+1) = pathAngles(end);
    pathStep = sqrt(mean(((x2-x1).^2+(y2-y1).^2)));
    pathCurvatures = wrapTo180(diff(pathAngles))/pathStep;
    pathCurvatures(end+1) = pathCurvatures(end);
    out.pathAngles = pathAngles;
    out.pathCurvatures = pathCurvatures;
    out.dtEstimate = pathStep/1;
    out.lenX = 4;
    out.lenU = 3;
end