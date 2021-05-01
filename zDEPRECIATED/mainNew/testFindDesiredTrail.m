clear
% path = load("paths/path11sin.mat").path;
% pathFile = "paths/path13roundedCorner.mat";
% pathFile = "paths/path10offset.mat";
% pathFile = "paths/path11sin.mat";
% pathFile = 'paths/pathProvidedLap.mat';
% pathFile = "paths/path16circle.mat";
pathFile = "paths/path19corner.mat";

path = load(pathFile).path;
config.L_t = 2;
config.L_h = 0.75;
config.L_i = 1;
config.v = -1;
x1 = path(1:end-1,1);
y1 = path(1:end-1,2);
x2 = path(2:end,1);
y2 = path(2:end,2);
pathStep = sqrt(mean((x2-x1).^2 + (y2-y1).^2));
config.pathDt = abs(pathStep/config.v);


t = findDesiredTrail3(config, path);

%% Plot vars
% figure(1), clf, hold on
% plot(t.x)
% plot(t.y)
% plot(t.theta)

%% Animate
wheelQuiver = 1;

xr = t.x';
yr = t.y';
theta = t.theta';
phi = t.phi';
delta_t = t.delta_t';
delta_i = 0*t.delta_i';
radialAccel = t.radialAccel';

xf = t.xf';
yf = t.yf';
uf = wheelQuiver*cos(theta + delta_t);
vf = wheelQuiver*sin(theta + delta_t);

ur = 5*radialAccel.*cos(theta + pi/2);
vr = 5*radialAccel.*sin(theta + pi/2);

xh = t.xh';
yh = t.yh';

xi = t.xi';
yi = t.yi';
ui = wheelQuiver*cos(theta + phi + delta_i);
vi = wheelQuiver*sin(theta + phi + delta_i);

circlesx = [xf;xr;xi];
circlesy = [yf;yr;yi];

pointsx = [xf;xr;xh;xi];
pointsy = [yf;yr;yh;yi];

quiverx = [xf;xf;xr;xi;xi];
quivery = [yf;yf;yr;yi;yi];
quiveru = [uf;-uf;ur;ui;-ui];
quiverv = [vf;-vf;vr;vi;-vi];

plotInitialised = false;
for j = 1:height(t)
% for j = height(t):-1:1
    if ~plotInitialised
        f = figure(1);
        clf, axis equal, hold on
        plot(xr,yr,'b')
        handle_circles = plot(circlesx(:,j)',circlesy(:,j)','.k','MarkerSize',20);
        handle_points = plot(pointsx(:,j)',pointsy(:,j)','.-k');
        handle_quiver = quiver(quiverx(:,j)',quivery(:,j)',quiveru(:,j)',quiverv(:,j)',1,'r','AutoScale','off');
%         handle_imp = plot(xi(1),yi(1),'-m');
        plot(xi,yi,'g');
%         plot(t.xi,t.yi,'--m');
        plotInitialised = true;
    else
        if ~ishghandle(f); break; end
        set(handle_circles,'XData',circlesx(:,j)','YData',circlesy(:,j)');
        set(handle_points,'XData',pointsx(:,j)','YData',pointsy(:,j)');
        set(handle_quiver,'XData',quiverx(:,j)','YData',quivery(:,j)','UData',quiveru(:,j)','VData',quiverv(:,j)','AutoScale','off');
    end
    pause(0.1)
end