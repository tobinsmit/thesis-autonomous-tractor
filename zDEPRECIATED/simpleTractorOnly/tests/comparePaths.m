clear
L = 2;
v = 0.00001;
config.L = L;
config.v = v;

x = 0;
y = 0;
theta = 0;
initX = [x; y; theta];

delta = 0;

t = 0;
XsAb = zeros(3,1000);
XsAb(:,1) = initX;
UsAb = zeros(2,1000);
XsDir(:,1) = initX;
UsDir = zeros(2,1000);
i = 1;
while t < 20
    disp(i)
    XiAb = XsAb(:,i);
    XiDir = XsDir(:,i);
    tNext = t + 0.1;
    tspan = [t tNext];
    
    UiAb = controllerSine(config,XiAb,t);
    UiDir = controllerSine(config,XiDir,t);
    UsAb(:,i) = UiAb;
    UsDir(:,i) = UiDir;
    
    [A,B] = getAB(L, XiAb, [0; v]);
    getDxAb = @(t,X) A*X+B*UiAb;
    
    getDxDir = @(t,X) plantDir(config, X, UiDir);

    [t,XnextAb] = ode45(getDxAb,tspan,XiAb);
    [t,XnextDir] = ode45(getDxDir,tspan,XiDir);
    XsAb(:,i+1) = XnextAb(end,:)';
    XsDir(:,i+1) = XnextDir(end,:)';
    i = i + 1;
    t = tNext;
end
XsAb = XsAb(:,1:i-1);
UsAb = UsAb(:,1:i-1);

XsDir = XsDir(:,1:i-1);
UsDir = UsDir(:,1:i-1);

figure(1), clf, hold on
title("Paths")
plot(XsAb(1,:),XsAb(2,:))
plot(XsDir(1,:),XsDir(2,:))
legend('AX+BU','Direct')

figure(2), clf, hold on
title("\theta vs time")
plot(XsAb(3,:))
plot(XsDir(3,:))
legend('AX+BU','Direct')

figure(3), clf, hold on
title("\delta vs time")
plot(UsAb(1,:))
plot(UsDir(1,:))
legend('AX+BU','Direct')

