% clear, close all
% path = load('pathSin1.mat').path;
x1 = path(1:end-1,1);
y1 = path(1:end-1,2);
x2 = path(2:end,1);
y2 = path(2:end,2);
dx = x2-x1;
dy = y2-y1;
angle = atan2d(dy,dx);
curvature = wrapTo180(diff(angle));

figure(1),clf
subplot(1,2,1)
h = plot([x1(1),x2(1)],[y1(1),y2(1)], 'ro');
axis equal
axis([minmax(path(:,1)'), minmax(path(:,2)')])


subplot(1,2,2)
plot(angle)
ylim([-360 360])
pause(0.01)

for i = 2:length(path)-1
    set(h, 'XData', [x1(i),x2(i)], 'YData', [y1(i),y2(i)]);
    sgtitle(num2str(i) + "/" + num2str(length(path)-1))
    drawnow
    pause(0.05)
end
