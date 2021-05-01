function plotPathTrail(path,tit,trail,config,cot)
    figure, clf, hold on, axis equal
    title(tit)
    
    xminmax = minmax(path(:,1)');
    yminmax = minmax(path(:,2)');
    xRange = diff(xminmax);
    yRange = diff(yminmax);
    defaultMatlabAspectRatio = 1.267893660531697;
    if xRange > yRange * defaultMatlabAspectRatio
        xlim(xminmax + [-2 2])
    else
        ylim(yminmax + [-2 2])
    end
    
    
    rateOfPathDots = ceil(0.3/getAveragePointSpace(path));
    plot(path(1:rateOfPathDots:end,1),path(1:rateOfPathDots:end,2),'.k')
    plot(trail.xr,trail.yr,'b')
    plot(trail.xi,trail.yi,'r')
    
%     sketchNum = input("Enter num of sketched vehicles:");
%     idxs = round(linspace(1,height(trail),sketchNum));
    relIdxs = input("Enter sketched vehicles positions (from 0-1):");
    idxs = round(relIdxs*(height(trail)-1))+1;
    for idx = idxs
        sketchVehicle(trail(idx,:),config)
    end
    
    legend("Set path","Tractor result","Implement result")
end

function sketchVehicle(frame, config)
    init = 0;
    if init
        disp("inital set up")
        config.handles = {};
        config.handles.fig = figure(123);
        clf
        hold on
        plot(config.path(:,1),config.path(:,2), '-');
        axis equal
        plot(config.trail(:,1),config.trail(:,2), 'k--');
        plot(config.path(1,1),config.path(1,2), 'g*');
        plot(config.path(end,1),config.path(end,2), 'r*');
        legend(["Target Path","Result Trail","Start","End"])
        xlim(minmax([config.path(:,1)', config.trail(:,1)']) + [-5 5])
        ylim(minmax([config.path(:,2)', config.trail(:,2)']) + [-5 5])
        box on
    end
    
    L_t = config.L_t;
    L_h = config.L_h;
    L_i = config.L_i;
    xr = frame.xr;
    yr = frame.yr;
    theta_t = frame.theta_t;
    phi = frame.phi;    
    delta_t = frame.delta_t;
    delta_i = frame.delta_i;
    xi = frame.xi;
    yi = frame.yi;
    xh = frame.xh;
    yh = frame.yh;
    xf = xr + config.L_t * cos(theta_t);
    yf = yr + config.L_t * sin(theta_t);

    % Tractor Rear wheel
    trWheel = rectFromHeightLength(0.2, 0.9);
    trWheel = rotatePointsOrigin(trWheel, theta_t);
    trWheel = translatePoints(trWheel, [xr yr]');

    % Tractor Front wheel
    tfWheel = rectFromHeightLength(0.2, 0.9);
    tfWheel = rotatePointsOrigin(tfWheel, delta_t);
    tfWheel = translatePoints(tfWheel, [L_t 0]');
    tfWheel = rotatePointsOrigin(tfWheel, theta_t);
    tfWheel = translatePoints(tfWheel, [xr yr]');
    config.tfWheelCentre = mean(tfWheel,2);

    % Hitch Point
    hitchPointCenter = [-L_h, 0]';
    hitchPointCenter = rotatePointsOrigin(hitchPointCenter, theta_t);
    hitchPointCenter = translatePoints(hitchPointCenter, [xr yr]');
    angles = linspace(0,2*pi,60);
    hitchPointPoints = hitchPointCenter + 0.1*[cos(angles); sin(angles)];

    % Implement wheel
    theta_i = theta_t + phi;
    iWheel = rectFromHeightLength(0.2, 0.9);
    iWheel = rotatePointsOrigin(iWheel, delta_i);
    iWheel = translatePoints(iWheel, [-L_i,0]');
    iWheel = rotatePointsOrigin(iWheel, theta_i);
    iWheel = translatePoints(iWheel, [xh;yh]);
    
    plot([xi xh xr xf],[yi yh yr yf],'k')
    patch(trWheel(1,:),trWheel(2,:),'w', 'HandleVisibility','off');
    patch(tfWheel(1,:),tfWheel(2,:),'w', 'HandleVisibility','off');
    path(hitchPointPoints(1,:),hitchPointPoints(2,:),'w', 'HandleVisibility','off');
    patch(iWheel(1,:),iWheel(2,:),'w', 'HandleVisibility','off');

%     plot([xi xh xr xf],[yi yh yr yf],'k')
%     plot(trWheel(1,:),trWheel(2,:),'k');
%     plot(tfWheel(1,:),tfWheel(2,:),'k');
%     plot(hitchPointPoints(1,:),hitchPointPoints(2,:),'k');
%     plot(iWheel(1,:),iWheel(2,:),'k');
end

function p = rectFromHeightLength(h,l)
    p = [
        -l/2 -h/2
        -l/2 h/2
        l/2 h/2
        l/2 -h/2
        -l/2 -h/2]';
end

function v = rotatePointsOrigin(v, theta)
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    v = R*v;
end

function v = translatePoints(v, d)
    v = v + d;
end

function d = getAveragePointSpace(path)
    dx = mean(abs(diff(path(:,1))));
    dy = mean(abs(diff(path(:,2))));
    d = sqrt(dx^2+dy^2);
end


