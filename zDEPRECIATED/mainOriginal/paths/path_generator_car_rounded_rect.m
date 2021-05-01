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
alphas(and(2 < times, times < 4)) = alphaTurning;
alphas(and(4 < times, times < 6)) = -alphaTurning;
alphas(and(8 < times, times < 10)) = alphaTurning;
alphas(and(10 < times, times < 12)) = -alphaTurning;
alphas(and(14 < times, times < 16)) = alphaTurning;
alphas(and(16 < times, times < 18)) = -alphaTurning;
alphas(and(20 < times, times < 22)) = alphaTurning;
alphas(and(22 < times, times < 24)) = -alphaTurning;

for t = 2:length(times)
    omegas(t) = omegas(t-1) + alphas(t)*dt;
    theta = theta + omegas(t)*dt;
    x = x + v * cosd(theta)*dt;
    y = y + v * sind(theta)*dt;
    path(t,:) = [x y];
end

figure(1), clf
plot(path(:,1), path(:,2), '+-')
axis equal

% figure(2), clf
% plot(omegas)