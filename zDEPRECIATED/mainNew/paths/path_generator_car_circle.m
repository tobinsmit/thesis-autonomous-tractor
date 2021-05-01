clear
alphaTurning = 23.75;

v = 2;
x = 0;
y = 0;
theta = 0;

L_t = 2;
L_h = 0.75;
L_i = 1;

tfinal = 7;
dt = 0.05;
times = 0:dt:tfinal;
path = zeros(length(times),2);
omegas = zeros(size(times));
alphas = zeros(size(times));


for t = 2:length(times)
    theta = v/L_t*tand(35)*times(t);
    x = x + v * cos(theta)*dt;
    y = y + v * sin(theta)*dt;
    path(t,:) = [x y];
end

figure(1), clf
plot(path(:,1), path(:,2), '+-')
axis equal

% figure(2), clf
% plot(omegas)