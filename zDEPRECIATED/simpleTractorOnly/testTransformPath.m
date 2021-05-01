ctrlr controllerMpcOdePrecalc;
pathFile = 'paths/path7';
path = load(pathFile).path;
config.path = path;
config.lenPath = length(path);
config.lenX = 3;
config.lenU = 2;
config.L = 2;
config.v = -1;
ctrlr = setup(ctrlr, config);

% initX = [1 1 pi]';
% thetas = linspace(-pi,pi,13);
% dXs = [
%     cos(thetas)
%     sin(thetas)
%     thetas
% ];

figure(1), clf
plot(squeeze(ctrlr.outputSeries(:,1,:))',squeeze(ctrlr.outputSeries(:,2,:))'), axis equal


ymin = min(min(ctrlr.outputSeries(:,2,:)));
ymax = max(max(ctrlr.outputSeries(:,2,:)));
for s = 1:length(ctrlr.outputSeries)
    figure(2), clf
    plot(squeeze(ctrlr.outputSeries(s,1,:))',squeeze(ctrlr.outputSeries(s,2,:))','o-')
    axis equal, xlim([-5 0]), ylim([ymin ymax])
    drawnow
    pause(0.1)
end

% outs = transformPath(ctrlr,dXs,initX);
% 
% figure(1)
% plot(outs(1,:),outs(2,:),'o'), axis equal
% hold on
% plot(initX(1),initX(2),'+');
