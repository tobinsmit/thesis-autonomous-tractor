function [U,ctrlrOut] = controllerMPCparallel(config, X, t, lastClosestPointIdx)
    %% Input
    x = X(1);
    y = X(2);
    thetaT = X(3) - 180;
    phi = X(4);


    %% Localise config variables
    path = config.path;
    horizon = config.horizon;
    inputOptions = config.inputOptions;
    lenX = config.lenX;
    lenU = config.lenU;

    %% Find spot on path
    [~,closestPointIdx] = findClosestPoint(config,X,lastClosestPointIdx);
    
    horizon = min(horizon,length(config.path)-closestPointIdx); % Make sure horizon doesn't go past end of path


    %% Simulate
    % Create new variables
    outputs = zeros(1,lenX,1);
    outputs(1,:,1) = X;
    inputs = zeros(1,lenU,0);
    
    % Simulate each horizon in parallel for each step
    for h = 1:horizon
        oldOutputs = outputs;
        oldInputs = inputs;
        numNewTests = size(oldOutputs,1)*size(inputOptions,1);
        newOutputs = zeros(numNewTests,lenX,h+1);
        newInputs  = zeros(numNewTests,lenU,h);
        
        % Continue each output trajectory
        for t = 1:size(oldOutputs,1)
            % Get history
            Xs = oldOutputs(t,:,:);
            Xs = reshape(Xs,[lenX,h]);
            lastX = oldOutputs(t,:,end);
            lastX = reshape(lastX,[lenX,1]);
            Us = oldInputs(t,:,:);
            Us = reshape(Us,[lenU,h-1]);
            
            % Run each input option
            for i = 1:size(inputOptions,1)
                U = inputOptions(i,:);
                U = reshape(U,[lenU,1]);
                dX = plant(config, lastX, U);
                newX = lastX + dX*config.dtEstimate;
                
                % Save output and input
                newXs = [Xs newX];
                newOutputs((t-1)*size(inputOptions,1) + i, :, :) = newXs;
                newInputs((t-1)*size(inputOptions,1) + i, :, :) = [Us U];
            end
        end
        outputs = newOutputs;
        inputs = newInputs;
    end
    
    % Run cost function
%     idxs = closestPointIdx:closestPointIdx+horizon;
%     targetPath = config.path(idxs,:)';
%     paths = outputs(:,1:2,:);
%     pathErrors = zeros(length(paths),1);
%     for p = 1:size(paths,1)
%         error = 0;
%         for h2 = 1:horizon
%             % Tractor
%             xt = paths(p,1,h);
%             yt = paths(p,2,h);
%             tdisSquared = (path(:,1) - xt).^2 + (path(:,2) - yt).^2;
%             error = error + sqrt(min(tdisSquared));
%             
%             % Implement
%             xi = paths(p,1,h) + config.Lh*cosd(thetaT) + config.Li*cosd(thetaT + phi);
%             yi = paths(p,2,h) + config.Lh*sind(thetaT) + config.Li*sind(thetaT + phi);
%             idisSquared = (path(:,1) - xi).^2 + (path(:,2) - yi).^2;
%             error = error + sqrt(min(idisSquared));
%         end
%         pathErrors(p) = error;
%     end
    
    % Run cost function
    idxs = closestPointIdx:closestPointIdx+horizon;
    targetPath = config.path(idxs,:)';
    paths = outputs(:,1:2,:);
    pathErrors = zeros(length(paths),1);
    for p = 1:size(paths,1)
        error = 0;
        for h2 = 1:horizon
            % Tractor
            xt = paths(p,1,h);
            yt = paths(p,2,h);
            [tdis, ~] = findClosestPoint(config, [xt;yt], closestPointIdx);
            error = error + tdis;
            
            % Implement
%             xi = xt + config.Lh*cosd(thetaT) + config.Li*cosd(thetaT + phi);
%             yi = yt + config.Lh*sind(thetaT) + config.Li*sind(thetaT + phi);
%             [idis, ~] = findClosestPoint(config, [xi;yi], closestPointIdx);
%             error = error + 0.5*idis;
        end
        pathErrors(p) = error;
    end
    
    
    [~,minErrIdx] = min(pathErrors);
    bestU = inputs(minErrIdx,:,1);
    bestU = reshape(bestU,[lenU,1]);
    pathChosen = outputs(minErrIdx,:,:);
    pathChosen = reshape(pathChosen,[lenX,horizon+1]);
    
    r = round(rand(500,1) * (length(paths)-1) + 1);
    ctrlrOut.options = paths(r,:);
    ctrlrOut.idxs = idxs;
    ctrlrOut.chosen = pathChosen;
    ctrlrOut.closestPointIdx = closestPointIdx;
    
    U = bestU;
end

