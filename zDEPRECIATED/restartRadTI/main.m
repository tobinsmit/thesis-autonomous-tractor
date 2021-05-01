clear
config.v = 1;
config.L_t = 2;
config.L_h = 0.75;
config.L_i = 1;

initX = [
    0 % x
    0 % y
    0 % theta
    0 % phi
];

dt = 0.01;
tTotal = 20;
times = 0:dt:tTotal;
nFrames = length(times);
Xs = zeros(4,nFrames);
Us = zeros(2,nFrames);
Xs(:,1) = initX;
t = 0;
for i = 2:nFrames
    t = times(i);
    X = Xs(:,i-1);
    U = controller(config,t,X);
    Us(:,i) = U;
    [~,Xpath] = ode45(@(t,X) plant(config,X,U),[0 dt],X);
    Xs(:,i) = Xpath(end,:)';
end

%% Animate

theta = Xs(3,:);
phi = Xs(4,:);
delta_t = Us(1,:);
delta_i = Us(2,:);
wheelsize = 0.25;

xr = Xs(1,:);
yr = Xs(2,:);

xf = xr + config.L_t*cos(theta);
yf = yr + config.L_t*sin(theta);
uf = cos(theta + delta_t);
vf = sin(theta + delta_t);

xh = xr - config.L_h*cos(theta);
yh = yr - config.L_h*sin(theta);

xi = xh - config.L_i*cos(theta + phi);
yi = yh - config.L_i*sin(theta + phi);
ui = cos(theta + phi + delta_i);
vi = sin(theta + phi + delta_i);

pointsx = [xf;xr;xh;xi];
pointsy = [yf;yr;yh;yi];
quiverx = [xf;xf;xi;xi];
quivery = [yf;yf;yi;yi];
quiveru = [uf;-uf;ui;-ui];
quiverv = [vf;-vf;vi;-vi];
plotInitialised = false;
for i = 1:10:nFrames
    if ~plotInitialised
        f = figure(1); clf
        plot(Xs(1,:),Xs(2,:),'b')
        axis equal
        hold on
        handle_points = plot(pointsx(:,i)',pointsy(:,i)','-ok');
        handle_quiver = quiver(quiverx(:,i)',quivery(:,i)',quiveru(:,i)',quiverv(:,i)',wheelsize,'r');
        plotInitialised = true;
    else
        if ~ishghandle(f); break; end
        set(handle_points,'XData',pointsx(:,i)','YData',pointsy(:,i)');
        set(handle_quiver,'XData',quiverx(:,i)','YData',quivery(:,i)','UData',quiveru(:,i)','VData',quiverv(:,i)');
    end
    drawnow
end







