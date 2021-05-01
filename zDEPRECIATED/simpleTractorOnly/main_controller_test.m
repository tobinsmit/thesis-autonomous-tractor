clear

% Choose controller and path
% ctrlr = controllerSine;
% ctrlr = controllerCompPID;
% ctrlr = controllerMpc;
% ctrlr = controllerSmc;
% ctrlr = controllerSmcSpline;
ctrlr = controllerMpcOdePrecalc;
% ctrlr = controllerMpcOdePrecalcSmoothed;
% ctrlr = controllerMpc2;
% pathFile = 'paths/path4'; % Offset line
pathFile = 'paths/path7';
% pathFile = 'paths/path9';

%% Setup up controller and path
path = load(pathFile).path;
% path = path(1:120,:);
config.path = path;
config.lenPath = length(path);
config.lenX = 4;
config.lenU = 2;
config.L = 2;
config.v = -1;
ctrlr = setup(ctrlr, config);

x1 = path(1:end-1,1);
y1 = path(1:end-1,2);
x2 = path(2:end,1);
y2 = path(2:end,2);
pathLength = sum(sqrt((x2-x1).^2+(y2-y1).^2));
pathTime = abs(pathLength/config.v);

%% Initialise state system variables
% Initial X
xp = 0;
yp = 2;
theta = pi;
phi = 0;
initX = [xp; yp; theta; phi];

t = 0;
dt = 0.1;
Xs = zeros(config.lenX,1000);
Xs(:,1) = initX;
Us = zeros(config.lenU,1000);
i = 1;
ctrlrOutHistoryCellVector = cell(0);
% while and(t < 50, ctrlr.thisHorizon > 1)
% while and(t < 50, ctrlr.closestPointIdx < config.lenPath - 50)
while t < pathTime
    disp(i)
    Xi = Xs(:,i);
    tNext = t + dt;
    tspan = [t tNext];
    
    [ctrlr, Ui, ctrlrOut] = loop(ctrlr,Xi,t);
    Us(:,i) = Ui(1:2);
    ctrlrOutHistoryCellVector{i} = ctrlrOut;

%     [A,B] = getAB(config.L, Xi, [0; config.v]);
%     getDx = @(t,X) A*X+B*Ui;

    getDx = @(t,X) plantDir(config,X,Ui);
    
    [~,Xpath] = ode45(getDx,tspan,Xi);
    Xs(:,i+1) = Xpath(end,:)';
    i = i + 1;
    t = tNext;
end

% return
%% Animate
clear options
Xs = Xs(:,1:i-1);
Us = Us(:,1:i-1);
ctrlrOutHistoryStruct = cell2mat(ctrlrOutHistoryCellVector);
ctrlrOutHistoryCellMatrix = struct2cell(ctrlrOutHistoryStruct);
wheelQuiver = 2;
if ~isempty(ctrlrOutHistoryStruct)
    cpArr = [ctrlrOutHistoryStruct.closestPointIdx];
    
    if isfield(ctrlrOutHistoryStruct(1),'options')
        options = ctrlrOutHistoryCellMatrix(3,:,:);
        options = reshape(options,[length(options) 1]);
    end
else
    cpArr = [];
end
brange = minmax([minmax(path(:,1)') minmax(Xs(1,:)) minmax(path(:,2)') minmax(Xs(2,:))]);
xrange = minmax([minmax(path(:,1)') minmax(Xs(1,:))]);
yrange = minmax([minmax(path(:,2)') minmax(Xs(2,:))]);
blimPlot(1) = brange(1) - 0.1*diff(brange);
blimPlot(2) = brange(2) + 0.1*diff(brange);
xlimPlot(1) = xrange(1) - 0.1*diff(xrange);
xlimPlot(2) = xrange(2) + 0.1*diff(xrange);
ylimPlot(1) = yrange(1) - 0.1*diff(yrange);
ylimPlot(2) = yrange(2) + 0.1* diff(yrange);
f = figure(2);
clf, hold on, axis equal
plot(path(:,1),path(:,2),'--')
plot(Xs(1,:),Xs(2,:),'--')
% ylim(blimPlot)
% xlim(blimPlot)
ylim(ylimPlot)
xlim(xlimPlot)
xp = Xs(1,1);
yp = Xs(2,1);
theta = Xs(3,1);
up = wheelQuiver * cos(theta);
vp = wheelQuiver * sin(theta);

xf = xp + config.L * cos(theta);
yf = yp + config.L * sin(theta);
delta = Us(1,1);
uf = wheelQuiver * cos(theta+delta);
vf = wheelQuiver * sin(theta+delta);

handle_p = plot(xp,yp,'+');
handle_f = plot(xf,yf,'+');
handle_q = quiver([xp,xf],[yp,yf],[up,uf],[vp,vf]);
handle_closestPoint = plot(path(cpArr(1),1),path(cpArr(1),2),'o');
if exist('options','var')
    o = options{1};
    pathXs = reshape(o(:,1,:),[size(o,1),size(o,3)]);
    pathYs = reshape(o(:,2,:),[size(o,1),size(o,3)]);
%     handle_options = plot(pathXs',pathYs','k');
end
for j = 1:i-1
    xp = Xs(1,j);
    yp = Xs(2,j);
    theta = Xs(3,j);
    up = wheelQuiver * cos(theta);
    vp = wheelQuiver * sin(theta);

    xf = xp + config.L * cos(theta);
    yf = yp + config.L * sin(theta);
    delta = Us(1,j);
    uf = wheelQuiver * cos(theta+delta);
    vf = wheelQuiver * sin(theta+delta);
    
    
    
    if ~ishghandle(f); break; end

    set(handle_p,'XData',xp,'YData',yp)
    set(handle_f,'XData',xf,'YData',yf)
    set(handle_q,'XData',[xp,xf],'YData',[yp,yf],'UData',[up,uf],'VData',[vp,vf])
    
    set(handle_closestPoint,'XData',path(cpArr(j),1),'YData',path(cpArr(j),2))
    if exist('options','var')
        o = options{j};
        pathXs = num2cell(reshape(o(:,1,:),[size(o,1),size(o,3)])',2);
        pathYs = num2cell(reshape(o(:,2,:),[size(o,1),size(o,3)])',2);
%         set(config.handles.options,'XData',subTimes,{'YData'},num2cell(options,2));
%         set(handle_options,{'XData'},pathXs,{'YData'},pathYs)
    end
    drawnow;
%     pause(max(0,dt-0.1))
%     title(config.pathCurvatures(cpArr(j)))
end

