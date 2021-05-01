clear
alphaTurning = 23.75;

v = 2;
x = 0;
y = 0;
theta = 0;

tfinal = 23;
dt = 0.01;
times = 0:dt:tfinal;
path = zeros(length(times),2);
omegas = zeros(size(times));
alphas = zeros(size(times));

for t = 2:length(times)
    omegas(t) = omegas(t-1) + alphas(t)*dt;
    theta = theta + omegas(t)*dt;
    theta = 30*sin(times(t));
    x = x + v * cosd(theta)*dt;
    y = y + v * sind(theta)*dt;
    path(t,:) = [x y];
end

figure(1), clf
plot(path(:,1), path(:,2), '+-')
axis equal

% figure(2), clf
% plot(omegas)