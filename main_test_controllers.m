clear

% Setup controllers
global ctrlrs
ctrlrs = {
%     controllerCompPIDdesTrail,      'Compensated Offset Model Controller'
    controllerCompPIDdesTrail2,     'COMC'
%     controllerSmc1comp,             'SMC1'
%     controllerSmc2,                 'SMC2'
%     controllerSmc3spline,           'SMC3'  %  BAD
    controllerSmc4curveComp,        'SMC'
%     controllerMpc,                  'MPC'
%     controllerMpc2,                 'MPC2 ' % Doesn't work
    controllerMpc3odePrecalc,       'MPC'
};
numCtrlrs = size(ctrlrs,1);

% Setup config
config.lenX = 4;
config.lenU = 2;
config.L_t = 2;
config.L_h = 0.75;
config.L_i = 1;
config.v = -1;


% Setup test paths
% pathFilePaths = ["pathProvidedLapCut","path11sin","path14offset2m","path17circle35deg"];
% pathFilePaths = ["pathProvidedLap"];
pathFilePaths = ["path14offset2m"];
% pathFilePaths = ["path11sin","path14offset2m","path17circle35deg"];

numPaths = length(pathFilePaths);
paths = cell(1,numPaths);
desTrails = cell(1,numPaths);
pathLengths = zeros(1,numPaths);
for p = 1:numPaths
    path = load("paths/" + pathFilePaths(p)).path;
    paths{p} = path;
    desTrails{p} = findDesiredTrail2(config,path);
    pathLengths(p) = length(path);
end

dt = 0.1;
Xinit = [0;0;pi;0];

%% Simulate
trails = cell(numCtrlrs,numPaths);

closeProgressHandles();
progressNow = 0;
progressTotal = sum(pathLengths)*numCtrlrs;
if exist('progressHandle','var') && ishandle(progressHandle), close(progressHandle), end
progressHandle = waitbar(progressNow/progressTotal, "Simulating");

for p = 1:numPaths
    for c = 1:numCtrlrs
        path = paths{p};
        des = desTrails{p};
        config.path = path;
        config.des = des;
        config.pathLen = pathLengths(p);
        
        configOff = config;
        skew = 1.2;
        configOff.L_t = 1/skew*config.L_t;
        configOff.L_h = skew*config.L_h;
        configOff.L_i = skew*config.L_i;
        configOff.des = findDesiredTrail2(configOff, path);
        
        ctrlr = ctrlrs{c,1};
        ctrlr = setup(ctrlr,configOff);
        
        states = Xinit;
        Us = zeros(config.lenU,5000);
        CPs = zeros(config.lenU,5000);
        f = 1;
        t = 0;
        % Simulate
        while canContinue(ctrlr)
            P_t = states(:,f);
            [ctrlr, U, ctrlrOut] = loop(ctrlr, P_t, t);
            
            % Control bias
%             U = U*1.5;
%             U = min(U, [pi/3; pi/3]);
%             U = max(U, -[pi/3; pi/3]);

            Us(:,f) = U;
            CPs(f) = ctrlrOut.closestPointIdx;
            getDx = @(t,X) plantDir(config,X,U);
            [~,Xpath] = ode45(getDx,[0 dt],P_t);
            states(:,f+1) = Xpath(end,:)';
            f = f + 1;
            t = t + dt;
            
            progressNow = ((p-1)*c + c)*mean(pathLengths) + CPs(f);
            waitbar(progressNow/progressTotal,progressHandle);
        end
        Us = Us(:,1:f-1);
        Us(:,end+1) = Us(:,end);
        CPs = CPs(:,1:f-1);
        CPs(:,end+1) = CPs(:,end);
        
        % Calculate implement positions
        P_t = states(1:2,:);
        theta_t = states(3,:);
        phi = states(4,:);
        theta_i = theta_t + phi;
        P_h = P_t - config.L_h * [cos(theta_t);sin(theta_t)];
        P_i = P_h - config.L_i * [cos(theta_i);sin(theta_i)];

        states(5:6,:) = P_i;
        xr = states(1,:)';
        yr = states(2,:)';
        theta_t = theta_t';
        phi = phi';
        P_t = P_t';
        xh = P_h(1,:)';
        yh = P_h(2,:)';
        xi = P_i(1,:)';
        yi = P_i(2,:)';
        delta_t = Us(1,:)';
        delta_i = Us(2,:)';
        trail = table(xr,yr,theta_t,phi,delta_t,delta_i,P_t,xh,yh,xi,yi);
        
        trails{c,p} = trail;
    end
end
closeProgressHandles();

% %% Save data
% for p = 1:numPaths
%     for c = 1:numCtrlrs
%         filename = "trails/ctrlr-" + ctrlrs{c,2} + "-path-" + pathFilePaths(p) + ".mat";
%         trail = trails{c,p};
%         save(filename,'trail');
%     end
% end
%% Calculate error
% Setup outputs
errTracDeviations = zeros(numCtrlrs,numPaths);
errImplDeviations = zeros(numCtrlrs,numPaths);
trailsExag = cell(numCtrlrs,numPaths);

closeProgressHandles();
progressNow = 0;
progressTotal = numPaths*numCtrlrs;
if exist('progressHandle','var') && ishandle(progressHandle), close(progressHandle), end
progressHandle = waitbar(progressNow/progressTotal, "Measuring error");

for p = 1:numPaths
    for c = 1:numCtrlrs
        path = paths{p};
        trail = trails{c,p};
        des = findDesiredTrail2(config, path);
        
        % Caluclate points
        P_t = trail.P_t';
%         P_t = trail(1:2,:);
%         P_i = trail(5:6,:);

        % Find tractor error
        [idxs,P_t_path,dissSquared_t] = findClosestSplinePointMat(path,P_t);
        e_t = abs(sqrt(dissSquared_t));
        errTracMean = mean(e_t);
        Xexag = P_t + sqrt(e_t).*(P_t - P_t_path);

        % Find implement error
        phis = trail.phi';
        fullPathIdxs = 1:height(des);
        phis_des = spline(fullPathIdxs, des.phi, idxs);
        e_phi = abs(phis_des - phis);
        errImplMean = mean(e_phi);

%         [~,P_i_path,dissSquared_i] = findClosestSplinePointMat(path,P_i);
%         e_i = sqrt(dissSquared_i);
%         errImplTotal = sum(e_i);

        % Save
        errTracDeviations(c,p) = errTracMean;
        errImplDeviations(c,p) = errImplMean;
        trailsExag{c,p} = Xexag;
        waitbar(progressNow/progressTotal,progressHandle);
        progressNow = progressNow + 1;        
    end
end
closeProgressHandles();

%% Print results
fprintf('\n');
for c = 1:numCtrlrs
    d = {ctrlrs{c,2}};
    for p = 1:numPaths
        d{end+1} = errTracDeviations(c,p);
        d{end+1} = errImplDeviations(c,p);
        d{end+1} = errTracDeviations(c,p) + errImplDeviations(c,p);
        d{end+1} = [c,p];
    end
    printDetails(d)
end

function str = getPlotPathTrailFuncStr(ctrlrIdx,pathIdx)
    global ctrlrs
    
    pathStr = ['paths{' num2str(pathIdx) '}'];
    trailStr = ['trails{' num2str(ctrlrIdx) ',' num2str(pathIdx) '}'];
%     trailExagStr = ['trailsExag{' num2str(ctrlrIdx) ',' num2str(pathIdx) '}'];
    titleStr = ['''' ctrlrs{ctrlrIdx,2} ''''];
    str = strjoin([
        "plotPathTrail("
        pathStr
        ","
        titleStr 
        ","
        trailStr
        ",config)"
    ]);
%     str = strjoin([        
%         "figure(1), clf"
%         ",hold on"
%         ",axis equal"
%         ",title("
%             titleStr
%             ")"
%         ",plot("
%             pathStr
%             "(1:2:end,1),"
%             pathStr
%             "(1:2:end,2),'.-k')"
%         ", plot("
%             trailStr
%             "(1,:),"
%             trailStr
%             "(2,:),'b')"
%         ", plot("
%             trailStr
%             "(5,:),"
%             trailStr
%             "(6,:),'r')"
%     ]','');
end
    
function printDetails(details)
    for row = 1:size(details,1)
        for col = 1:size(details,2)
            data = details{row,col};
            if isa(data,'char')
                fprintf('% 25s ',data);
            elseif isa(data,'double') && length(data) == 1
                fprintf('% 6.3f ',data);
            elseif isa(data,'double') && length(data) == 2
                ctrlrIdx = data(1);
                pathIdx = data(2);
                fs = getPlotPathTrailFuncStr(ctrlrIdx,pathIdx);
                fprintf('<a href="matlab:%s">plot</a>', fs);
            end
        end
        fprintf('\n')
    end
end

function closeProgressHandles()
    delete(findall(0,'type','figure','tag','TMWWaitbar'))
end


