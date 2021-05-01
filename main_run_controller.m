clear

% Choose controller and path
% ctrlr = controllerSine;
% ctrlr = controllerCompPID;
% ctrlr = controllerCompPIDdesTrail;
% ctrlr = controllerCompPIDdesTrail2;
% ctrlr = controllerSmc1basic;
% ctrlr = controllerSmc2;
% ctrlr = controllerSmc3spline;
% ctrlr = controllerSmc4curveComp;
% ctrlr = controllerMpc1;
% ctrlr = controllerMpc2;
ctrlr = controllerMpc3odePrecalc;
% ctrlr = controllerMpcOdePrecalcSmoothed;

% pathFile = 'paths/path4.mat'; % Offset line
% pathFile = 'paths/path11sin.mat';
% pathFile = 'paths/path8roundedSquare';
pathFile = "paths/path10offset3m.mat";
% pathFile = "paths/path14offset2m.mat";
% pathFile = "paths/path15offset1m.mat";
% pathFile = "paths/path13roundedCorner.mat";
% pathFile = 'paths/pathProvidedLap.mat';
% pathFile = 'paths/pathProvidedLapCut.mat';
% pathFile = 'paths/path16circle.mat';
% pathFile = 'paths/path17circle35deg.mat';
% pathFile = 'paths/path18zigzag.mat';
% pathFile = 'paths/path19corner.mat';

lenX = 4;
lenU = 2;
L_t = 2;
L_h = 0.75;
L_i = 1;
v = -1;

%% Setup up controller and path
path = load(pathFile).path;
% path = path(1:120,:);
config.path = path;
config.pathLen = length(path);
config.lenX = lenX;
config.lenU = lenU;
config.L_t = L_t;
config.L_h = L_h;
config.L_i = L_i;
config.v = v;
config.des = findDesiredTrail2(config, path);

x1 = path(1:end-1,1);
y1 = path(1:end-1,2);
x2 = path(2:end,1);
y2 = path(2:end,2);
pathLength = sum(sqrt((x2-x1).^2+(y2-y1).^2));
pathTime = abs(pathLength/v);

%% Test specific vars
configOff = config;
c = 1.2;
configOff.L_t = 1/c*config.L_t;
configOff.L_h = c*config.L_h;
configOff.L_i = c*config.L_i;
configOff.v = config.v;
configOff.des = findDesiredTrail2(configOff, path);

%% Initialise state system variables
ctrlr = setup(ctrlr, config);

% Initial X
xp = 0;
yp = 0;
theta = pi;
phi = 0;
% xp = config.des.x(1);
% yp = config.des.y(1);
% theta = config.des.theta(1);
% phi = 0;

initX = [xp; yp; theta; phi];

t = 0;
dt = 0.1;
Xs = zeros(lenX,1);
Us = zeros(lenU,0);
Xs(:,1) = initX;
i = 1;
ctrlrOutCells = cell(0,0);

closeProgressHandles();
progressNow = 0;
progressTotal = config.pathLen;
if exist('progressHandle','var') && ishandle(progressHandle), close(progressHandle), end
progressHandle = waitbar(progressNow/progressTotal, "Simulating");
while canContinue(ctrlr) && progressNow < progressTotal && t < pathTime*2
%     disp(i)
    X = Xs(:,i);
    tNext = t + dt;
    tspan = [t tNext];
    
    [ctrlr, U, ctrlrOut] = loop(ctrlr,X,t);
%     U = U*1.5;
%     U = min(U, [pi/3; pi/3]);
%     U = max(U, -[pi/3; pi/3]);

    Us(:,i) = U(1:2);
    ctrlrOutCells{i} = ctrlrOut;

%     [A,B] = getAB(config.L, Xi, [0; v]);
%     getDx = @(t,X) A*X+B*Ui;

    getDx = @(t,X) plantDir(config,X,U);
    
    [~,Xpath] = ode45(getDx,tspan,X);
    Xs(:,i+1) = Xpath(end,:)';
    i = i + 1;
    t = tNext;
    progressNow = ctrlrOut.closestPointIdx;
    waitbar(progressNow/progressTotal,progressHandle);
end
%%
closeProgressHandles();

Xs = Xs(:,1:i-1); % Get rid of next X without U

ctrlrOutTable = struct2table(cell2mat(ctrlrOutCells));
cot = ctrlrOutTable;

P_t = Xs(1:2,:);
theta_t = Xs(3,:);
phi = Xs(4,:);
theta_i = theta_t + phi;
P_h = P_t - config.L_h * [cos(theta_t);sin(theta_t)];
P_i = P_h - config.L_i * [cos(theta_i);sin(theta_i)];

xr = Xs(1,:)';
yr = Xs(2,:)';
theta_t = theta_t';
phi = phi';
P_t = P_t';
xh = P_h(1,:)';
yh = P_h(2,:)';
xi = P_i(1,:)';
yi = P_i(2,:)';
delta_t = Us(1,:)';
delta_i = Us(2,:)';
trail = table(xr,yr,theta_t,phi,delta_t,delta_i,P_t,xh,yh,xi,yi,cot);

% plotPathTrail(path,"MPC Output Options",trail,config,cot);

% return
%% Animate
clear options
wheelQuiver = 0.5;

theta = Xs(3,:);
phi = Xs(4,:);
delta_t = Us(1,:);
delta_i = Us(2,:);
wheelsize = 0.25;
CPs = cot.closestPointIdx;

xr = Xs(1,:);
yr = Xs(2,:);

xf = xr + L_t*cos(theta);
yf = yr + L_t*sin(theta);
uf = wheelQuiver*cos(theta + delta_t);
vf = wheelQuiver*sin(theta + delta_t);

xh = xr - L_h*cos(theta);
yh = yr - L_h*sin(theta);

xi = xh - L_i*cos(theta + phi);
yi = yh - L_i*sin(theta + phi);
ui = wheelQuiver*cos(theta + phi + delta_i);
vi = wheelQuiver*sin(theta + phi + delta_i);

circlesx = [xf;xr;xi];
circlesy = [yf;yr;yi];
pointsx = [xf;xr;xh;xi];
pointsy = [yf;yr;yh;yi];
quiverx = [xf;xf;xi;xi];
quivery = [yf;yf;yi;yi];
quiveru = [uf;-uf;ui;-ui];
quiverv = [vf;-vf;vi;-vi];
plotInitialised = false;

filename = 'testAnimated-MPC-off3.gif';
p=1;

for j = 1:i-1
    if ~plotInitialised
        f = figure(10);
        clf, axis equal, hold on
%         h1 = plot(xi,yi,'r');
%         plot(config.des.xi,config.des.yi,'--g');
%         h2 = plot(Xs(1,:),Xs(2,:),'b');
        h3 = plot(path(:,1),path(:,2),'--b');
        handle_circles = plot(circlesx(:,j)',circlesy(:,j)','.k','MarkerSize',20);
        handle_points = plot(pointsx(:,j)',pointsy(:,j)','.-k');
        handle_cp = plot(path(CPs(j),1),path(CPs(j),2),'o');
        handle_quiver = quiver(quiverx(:,j)',quivery(:,j)',quiveru(:,j)',quiverv(:,j)',1,'r','AutoScale','off');
%         legend('Actual impl path','Target impl path','Actual trac path','Target trac path','Wheel','Body','Closest point','Wheel direction','Location','northwest')
        xlim(xlim)
        ylim(ylim)
%         title("MPC")
%         legend("Path","Wheels"
        plotInitialised = true;
    else
        if ~ishghandle(f); break; end
%         delete(p)
%         delete(q)
        set(handle_circles,'XData',circlesx(:,j)','YData',circlesy(:,j)');
        set(handle_points,'XData',pointsx(:,j)','YData',pointsy(:,j)');
        set(handle_cp,'XData',path(CPs(j),1),'YData',path(CPs(j),2));        
        set(handle_quiver,'XData',quiverx(:,j)','YData',quivery(:,j)','UData',quiveru(:,j)','VData',quiverv(:,j)','AutoScale','off');
    end
    
%     tracPathChosen = cot.trac_chosen{j};
%     tracPathOptions = cot.trac_options{j}(1:10:end,:,:);
%     q = plot(squeeze(tracPathOptions(:,1,:))',squeeze(tracPathOptions(:,2,:))','Color',[0.6 0.6 0.6]);
%     p = plot(squeeze(tracPathChosen(1,:))',squeeze(tracPathChosen(2,:))','g','LineWidth',3);
% %     uistack(h1,'top');
% %     uistack(h2,'top');
%     uistack(h3,'top');
%     uistack(handle_circles,'top');
%     uistack(handle_points,'top');
%     uistack(handle_cp,'top');
%     uistack(handle_quiver,'top');

    drawnow
%     pause(0.1)

    % Capture the plot as an image 
    frame = getframe(f); 
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256); 
    % Write to the GIF File 
    if j == 1 
      imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
    else 
      imwrite(imind,cm,filename,'gif','WriteMode','append'); 
    end 

end

%% Plot SMC
return
if any(ismember(cot.Properties.VariableNames,'e_t'))
    figure(2), clf
    x = cot.e_t';
    y = cot.ed_t';
    u = gradient(x);
    v = gradient(y);
%     quiver(x,y,u,v,'AutoScale','off')
    axis equal
    hold on
    r = minmax(x);
    f = figure(123);
    clf,  hold on, axis equal, grid on
    plot([-10 10],[10 -10],'k')
    xlim([-0.5 2.5])
    ylim([-1.5 1.5])
    title("SMC")
    xlabel("e")
    ylabel("e dot")
    plot(0,0,'ok')
    h2 = plot(x(1),y(1),'g','LineWidth',2);
    h1 = plot(x(1),y(1),'.b','MarkerSize',20);
    imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
    for j = 2:i-1
        set(h1, 'XData',x(j),'YData',y(j));
        set(h2, 'XData',x(1:j),'YData',y(1:j));
        drawnow
        pause(0.05)
        
        frame = getframe(f); 
        im = frame2im(frame); 
        [imind,cm] = rgb2ind(im,256); 
        % Write to the GIF File 
        imwrite(imind,cm,filename,'gif','WriteMode','append'); 
    end
    
end


function closeProgressHandles()
    delete(findall(0,'type','figure','tag','TMWWaitbar'))
end



