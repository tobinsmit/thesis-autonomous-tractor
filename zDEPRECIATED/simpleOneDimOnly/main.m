clear, dbstop if error

initialiser = @initialiseMPCseries;
controller = @steerMPCseries;


dt = 0.1;
totalTime = 2;
times = 0:dt:totalTime;
path = times + cos(times*pi);

X_init = [
    0 % y
    0 % vel
];

U = [
    0 % acc
];

config.dt = dt;
config.times = times;
config.path = path;
config.lenX = length(X_init);
config.lenU = length(U);

config = initialiser(config);

X = X_init;
X_history = zeros(length(X),length(times));
X_history(:,1) = X;
t = tic();
for f = 1:length(times)-1
    display([num2str(f) '/' num2str(length(times)-1)])
    
    [U,ctrlrOut] = controller(config, X, f);
    func = @(t,x) plant(config, x, U);
    [~,x] = ode45(func, [0 config.dt], X);
    X = x(end,:)';
    X_history(:,f+1) = X;
        
    config = updatePlot(...
        config,...
        X_history(1,1:f+1),...
        ctrlrOut.subTimes,...
        ctrlrOut.options,...
        ctrlrOut.chosenOutput...
    );
    pause()
end
toc(t)
updatePlot(config, X_history(1,:));
