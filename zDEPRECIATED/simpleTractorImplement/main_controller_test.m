clear

% Choose controller and path
% ctrlr = controllerSmc;
% ctrlr = controllerSimp;
ctrlr = controllerMpcOdePrecalc;
% pathFile = 'paths/path4'; % Offset line
pathFile = 'paths/path7';
% pathFile = 'paths/path9';

%% Setup up controller and path
path = load(pathFile).path;
% path = path(1:120,:);
% path = path(1:10,:);
config.path = path;
config.lenPath = length(path);
config.lenX = 3;
config.lenU = 2;
config.L_t = 2;
config.L_h = 0.75;
config.L_i = 1;
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
phi = pi/8;
initX = [xp; yp; theta; phi];

t = 0;
dt = 0.1;
Xs = zeros(4,1000);
Xs(:,1) = initX;
Us = zeros(2,1000);
CPs = zeros(1,1000);
i = 1;
% while and(t < 50, ctrlr.thisHorizon > 1)
% while and(t < 50, ctrlr.closestPointIdx < config.lenPath - 50)
while t < pathTime
% while t < 20
    disp(i)
    X = Xs(:,i);
    tNext = t + dt;
    tspan = [t tNext];
    
    [ctrlr, U, ctrlrOut] = loop(ctrlr,X,t);
    Us(:,i) = U;
    CPs(i) = ctrlrOut.closestPointIdx;

    getDx = @(t,X) plantDir(config,X,U);
    
    [~,Xpath] = ode45(getDx,tspan,X);
    Xs(:,i+1) = Xpath(end,:)';
    i = i + 1;
    t = tNext;
end


%% Animate
clear options
Xs = Xs(:,1:i-1);
Us = Us(:,1:i-1);
CPs = CPs(1:i-1);

wheelQuiver = 0.5;
plotInitialised = false;

for j = 1:i-1
    theta = Xs(3,j);
    phi = Xs(4,j);
    delta_t = Us(1,j);
    delta_i = Us(2,j);

    xp = Xs(1,j);
    yp = Xs(2,j);
    up = cos(theta);
    vp = sin(theta);

    xf = xp + config.L_t * cos(theta);
    yf = yp + config.L_t * sin(theta);
    uf = wheelQuiver*cos(theta+delta_t);
    vf = wheelQuiver*sin(theta+delta_t);
    
    xh = xp - config.L_h * cos(theta);
    yh = yp - config.L_h * sin(theta);
    uh = 0;
    vh = 0;
    
    xi = xh - config.L_i * cos(theta + phi);
    yi = yh - config.L_i * sin(theta + phi);
    ui = wheelQuiver*cos(theta + phi + delta_i);
    vi = wheelQuiver*sin(theta + phi + delta_i);
    
    
    
    if ~plotInitialised
        xrange = minmax([minmax(path(:,1)') minmax(Xs(1,:))]);
        yrange = minmax([minmax(path(:,2)') minmax(Xs(2,:))]);
        xlimPlot(1) = xrange(1) - 2;
        xlimPlot(2) = xrange(2) + 2;
        ylimPlot(1) = yrange(1) - 2;
        ylimPlot(2) = yrange(2) + 2;
        f = figure(2);
        clf, hold on, axis equal
        plot(path(:,1),path(:,2),'--g')
        plot(Xs(1,:),Xs(2,:),'--b')
        ylim(ylimPlot)
        xlim(xlimPlot)

%         handles.p = plot([xp,xf,xh,xi],[yp,yf,yh,yi],'+');
        handles.p = plot([xp,xf,xh,xi],[yp,yf,yh,yi],'-*k');
%         handles.q = quiver([xp,xf,xh,xi],[yp,yf,yh,yi],[up,uf,uh,ui],[vp,vf,vh,vi]);
        handles.q = quiver([xf,xf,xi,xi],[yf,yf,yi,yi],[uf,-uf,ui,-ui],[vf,-vf,vi,-vi],'r','AutoScale',false);
        handles.cp = plot(path(CPs(j),1),path(CPs(j),2),'sb');
        plotInitialised = 1;
    else
        if ~ishghandle(f); break; end
        
%         set(handles.p,'XData',[xp,xf,xh,xi],'YData',[yp,yf,yh,yi])
        set(handles.p,'XData',[xp,xf,xh,xi],'YData',[yp,yf,yh,yi])
%         set(handles.q,'XData',[xp,xf,xh,xi],'YData',[yp,yf,yh,yi],'UData',[up,uf,uh,ui],'VData',[vp,vf,vh,vi])
        set(handles.q,'XData',[xf,xf,xi,xi],'YData',[yf,yf,yi,yi],'UData',[uf,-uf,ui,-ui],'VData',[vf,-vf,vi,-vi])
        set(handles.cp,'XData',path(CPs(j),1),'YData',path(CPs(j),2))
    end
    pause(0.1);
    drawnow;
end

