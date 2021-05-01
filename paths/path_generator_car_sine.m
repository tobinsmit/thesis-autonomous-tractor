clear

L_t = 2;
L_h = 0.75;
L_i = 1;
v = -1;
config.L_t = L_t;
config.L_h = L_h;
config.L_i = L_i;
config.v = v;

x = 0;
y = 0;
theta = pi;
X = [x; y; theta; 0];
    
tturn = 0.1;

t = 0;
dt = 0.05;
tfinal = 30;
times = 0:dt:tfinal;
path = zeros(length(times),2);
delta_ts = zeros(size(times));
delta_is = zeros(size(times));
for i = 2:length(times)
    
    delta_ts(i) = heaviside(times(i)-tturn)*deg2rad(40)*sin((times(i)-tturn)*0.2);
    delta_is(i) = 0;
    U = [delta_ts(i); delta_is(i)];

    getDx = @(t,X) plantDir(config,X,U);
    
    tspan = [0 dt];
    [~,Xpath] = ode45(getDx,tspan,X);
    X = Xpath(end,:)';
    path(i,:) = X([1 2]);
    i = i + 1;
    t = t + dt;
end

figure(1), clf
plot(path(:,1), path(:,2), '+-')
axis equal

% figure(2), clf
% plot(omegas)