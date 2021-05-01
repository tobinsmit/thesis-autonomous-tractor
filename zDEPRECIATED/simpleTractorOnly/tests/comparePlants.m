clear

config.L = 2;
config.v = 1;

X = zeros(3,100);
X(1,:) = 0;
X(2,:) = 0;
% X(3,:) = pi/12;
X(3,:) = linspace(-pi,pi,100);


U = zeros(2,100);
U(1,:) = linspace(-pi/6,pi/6,100);
U(2,:) = 1;

dXdir = plantDir(config,X,U);
dXab = plantAB(config,X,U);

%% Plot

names = {'dx vs \theta', 'dy vs \theta', 'd\theta vs \delta'};
xtitles = {'\theta','\theta','\delta'};
ytitles = {'dx','dy','d\theta/d\delta'};
for i = 1:3
    figure(i), clf, hold on
    x = X(3,:);
%     x = U(1,:);
    plot(x,dXdir(i,:),'--b');
    plot(x,dXab(i,:),'b');
    title(names{i})
    legend('f(X,U)','A(X,U)*X+B(X,U)*U')
    xlim(minmax(x))
    xlabel(xtitles{i})
    ylabel(ytitles{i})
end
