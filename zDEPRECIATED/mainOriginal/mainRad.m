clear
close all
pathFile = 'paths/pathSin1.mat';
% pathFile = 'paths/path9.mat';
controller = @controller2rad;
% controller = @controllerSine;
% controller = @controllerMPCparallel;


path = load(pathFile).path;
% path = path(1:1000,:);
config.controller = controller;
config.path = path;
config.Lt = 2;
config.Lh = 0.75;
config.Li = 1;
config.dims = [config.Lt; config.Lh; config.Li];

x1 = path(1:end-1,1);
y1 = path(1:end-1,2);
x2 = path(2:end,1);
y2 = path(2:end,2);
dx = x2-x1;
dy = y2-y1;
pathAngles = atan2(dy,dx);
pathAngles(end+1) = pathAngles(end);
pathStep = sqrt(mean(((x2-x1).^2+(y2-y1).^2)));
pathCurvatures = wrapToPi(diff(pathAngles))/pathStep;
pathCurvatures(end+1) = pathCurvatures(end);
config.pathAngles = pathAngles;
config.pathCurvatures = pathCurvatures;
config.dtEstimate = pathStep/1;
config.lenX = 4;
config.lenU = 3;

config = initialiseMPC(config);

x = path(1,1);
y = path(1,2);
thetaT = pathAngles(1)+pi;
phi = 0;

initX = [
    x
    y
    thetaT
    phi
];

dt = 0.01;

startTic = tic();
Xs = initX;
i = 1;
t = 0;
tFinalEstimate = length(path) * config.dtEstimate;
closestPointIdx = 1;
[Us,~] = controller(config,initX,t,closestPointIdx);
% while (closestPointIdx < length(path)-1) && (t < tFinalEstimate*1.2)
while toc(startTic) < 0.5
    Xi = Xs(:,end);
    [Ui, ctrlrOut] = controller(config,Xi,t,closestPointIdx);
    Us(:,end+1) = Ui;
    closestPointIdx = ctrlrOut.closestPointIdx;
    
    % Basic
%     dX = plantRadWithAB(config, Xi, Ui);
%     newX = Xi + dt * dX;
    
    % ODE45
%     getDX = @(t,X) plantRadWithAB(config, Xi, Ui);
    getDX = @(t,X) plantRadWithAB(config, Xi, Ui);
    [~,x] = ode45(getDX, [0 config.dtEstimate], Xi);
    newX = x(end,:)';

    Xs(:,end+1) = newX;
    display([num2str(closestPointIdx) '/' num2str(length(path))])
    t = t + dt;
%     [config,handles] = plotModel(config, Xi, handles);
end
toc(startTic);

% Plot path
% figure(534)
% plot(X(:,1), X(:,2), 'o-')

%% Animate
if isfield(config, 'handles') && isfield(config.handles, 'fig')
    config = rmfield(config, 'handles');
end
config.trail = Xs(:,1:2);
times = 0:dt:dt*length(Xs);
% for f = 1:1/dt/5:length(Xs)
for f = 1:length(Xs)
    if isfield(config, 'handles') && ~ishghandle(config.handles.fig)
        break
    end
    config = plotModel(config, Xs(:,f), Us(:,f));
    drawnow;
end

%%
% figure(2)
% pathangles = zeros(1,length(rs));
% calcangles = zeros(1,length(rs));
% for i = 1:length(rs)
%     pathangles(i) = rs{i}.pathangle;
%     calcangles(i) = rs{i}.deltaPoint;
% end
% plot(pathangles)
% hold on
% plot(calcangles,'.')


%% Plot functions

function config = plotModel(config, X, U)
    if isfield(config,'handles')
        init = 0;
    else
        init = 1;
    end
    
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
    
    Lt = config.Lt;
    Lh = config.Lh;
    Li = config.Li;
    x = X(1);
    y = X(2);
    thetaT = X(3);
    phi = X(4);
    
%     U = controller(config, X, t);
    
    deltaT = U(2);
    deltaI = U(3);

    % Tractor Rear wheel
    trWheel = rectFromHeightLength(0.2, 0.9);
    trWheel = rotatePointsOrigin(trWheel, thetaT);
    trWheel = translatePoints(trWheel, [x y]');

    % Tractor Front wheel
    tfWheel = rectFromHeightLength(0.2, 0.9);
    tfWheel = rotatePointsOrigin(tfWheel, deltaT);
    tfWheel = translatePoints(tfWheel, [Lt 0]');
    tfWheel = rotatePointsOrigin(tfWheel, thetaT);
    tfWheel = translatePoints(tfWheel, [x y]');
    config.tfWheelCentre = mean(tfWheel,2);

    % Hitch Point
    hitchPoint = [-Lh, 0]';
    hitchPoint = rotatePointsOrigin(hitchPoint, thetaT);
    hitchPoint = translatePoints(hitchPoint, [x y]');

    % Implement wheel
    thetaI = thetaT + phi;
    iWheel = rectFromHeightLength(0.2, 0.9);
    iWheel = rotatePointsOrigin(iWheel, deltaI);
    iWheel = translatePoints(iWheel, [Li,0]');
    iWheel = rotatePointsOrigin(iWheel, thetaI);
    iWheel = translatePoints(iWheel, hitchPoint(:,end));
    
    if init
        config.handles.trWheel = patch(trWheel(1,:),trWheel(2,:),'w', 'HandleVisibility','off');
        config.handles.tfWheel = patch(tfWheel(1,:),tfWheel(2,:),'w', 'HandleVisibility','off');
        config.handles.carPoints = plot(hitchPoint(1,:),hitchPoint(2,:),'ko', 'HandleVisibility','off');
        config.handles.iWheel = patch(iWheel(1,:),iWheel(2,:),'w', 'HandleVisibility','off');
        disp("inital plotting")
    else
        if ishghandle(config.handles.fig)
            figure(123);
            set(config.handles.trWheel,'XData',trWheel(1,:),'YData',trWheel(2,:));
            set(config.handles.tfWheel,'XData',tfWheel(1,:),'YData',tfWheel(2,:));
            set(config.handles.carPoints,'XData',hitchPoint(1,:),'YData',hitchPoint(2,:));
            set(config.handles.iWheel,'XData',iWheel(1,:),'YData',iWheel(2,:));
        end
    end
end

function p = rectFromHeightLength(h,l)
    p = [
        -l/2 -h/2
        -l/2 h/2
        l/2 h/2
        l/2 -h/2]';
end

function v = rotatePointsOrigin(v, theta)
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    v = R*v;
end

function v = translatePoints(v, d)
    v = v + d;
end
