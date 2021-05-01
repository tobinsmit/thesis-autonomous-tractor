clear
cnf.TOTAL_NUM_POINTS = 1000;
cnf.DISTANCE_BETWEEN_POINTS = 0.2;
cnf.MAX_OMEGA = 20;
cnf.MAX_ALPHA = 3;

points = [
    -cnf.DISTANCE_BETWEEN_POINTS 0
    0 0
    cnf.DISTANCE_BETWEEN_POINTS 0
];

figure(1), clf
handle = plot(points(:,1), points(:,2),'*-');
axis([-15 15 -15 15])
title(num2str(size(points,1)) + "/" + num2str(cnf.TOTAL_NUM_POINTS))

while (size(points,1) < cnf.TOTAL_NUM_POINTS)
    input = ginput(1);
    newPoint = getAllowablePoint(points, input, cnf);
    points = [points; newPoint];
    title(num2str(size(points,1)) + "/" + num2str(cnf.TOTAL_NUM_POINTS))
    set(handle, 'XData', points(:,1), 'YData', points(:,2));
end


function point = getAllowablePoint(points, input, cnf)
    dpoints = diff(points);
    thetas = atan2d(dpoints(:,2),dpoints(:,1));
    thetaLast = thetas(end);
    omegas = diff(thetas);
    omegaLast = omegas(end);
    omegaLeftMax = omegaLast - cnf.MAX_ALPHA;
    omegaLeftMax = constrain(omegaLeftMax, -cnf.MAX_OMEGA, cnf.MAX_OMEGA);
    omegaRightMax = omegaLast + cnf.MAX_ALPHA;
    omegaRightMax = constrain(omegaRightMax, -cnf.MAX_OMEGA, cnf.MAX_OMEGA);
    thetaLeftMax = thetaLast + omegaLeftMax;
    thetaRightMax = thetaLast + omegaRightMax;
    pointInput = input - points(end,:);
    thetaInput = atan2d(pointInput(2),pointInput(1));
    theta = constrainAngle(thetaInput, thetaLeftMax, thetaRightMax);
    point = points(end,:) + cnf.DISTANCE_BETWEEN_POINTS*[cosd(theta) sind(theta)];
end

function x = constrain(x, minx, maxx)
    x = max(x, minx);
    x = min(x, maxx);
end

function x = constrainAngle(x, minT, maxT)
    x = wrapTo360(x);
    minT = wrapTo360(minT);
    maxT = wrapTo360(maxT);
    if minT < maxT
        x = max(x, minT);
        x = min(x, maxT);
    else
        if maxT < x && x < minT
            angleToMinT = min(wrapTo360([minT - x, x - minT]));
            angleToMaxt = min(wrapTo360([maxT - x, x - maxT]));
            if angleToMinT < angleToMaxt
                x = minT;
            else
                x = maxT;
            end
        end
    end
end