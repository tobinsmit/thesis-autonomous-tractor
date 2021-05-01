function [U,ctrlrOut] = steerMPC(config, X, thisF)
    % Constants
    horizon = config.horizon;
    horizon = min(horizon,length(config.path)-thisF);
    inputOptions = config.U_options;
    lenX = config.lenX;
    lenU = config.lenU;
        
    % Create new variables
    outputs = zeros(1,lenX,1);
    outputs(1,:,1) = X;
    inputs = zeros(1,config.lenU,0);
    
    % Simulate
    for f = 1:horizon
        oldOutputs = outputs;
        oldInputs = inputs;
        numNewTests = size(oldOutputs,1)*size(inputOptions,1);
        newOutputs = zeros(numNewTests,lenX,f+1);
        newInputs  = zeros(numNewTests,lenU,f);
        
        % Continue each output trajectory
        for i = 1:size(oldOutputs,1)
            % Get history
            Xs = oldOutputs(i,:,:);
            Xs = reshape(Xs,[lenX,f]);
            X = oldOutputs(i,:,end);
            X = reshape(X,[lenX,1]);
            Us = oldInputs(i,:,:);
            Us = reshape(Us,[lenU,f-1]);
            
            % Run each input option
            for o = 1:size(inputOptions,1)
                U = inputOptions(o,:);
                U = reshape(U,[lenU,1]);
                dX = plant(config, X, U);
                newX = X + dX*config.dt;
                
                % Save output and input
                newXs = [Xs newX];
                newOutputs((i-1)*size(inputOptions,1) + o, :, :) = newXs;
                newInputs((i-1)*size(inputOptions,1) + o, :, :) = [Us U];
            end
        end
        outputs = newOutputs;
        inputs = newInputs;
    end
    
    % Run cost function
    idxs = thisF:thisF+horizon;
    targetPath = config.path(idxs);
    paths = outputs(:,1,:);
    paths = reshape(paths,[size(outputs,1),horizon+1]);
    pathErrors = abs(paths - targetPath);
    cumErrors = sum(pathErrors,2);
    [~,minErrIdx] = min(cumErrors);
    bestU = inputs(minErrIdx,:,1);
    bestU = reshape(bestU,[lenU,1]);
    pathChosen = outputs(minErrIdx,:,:);
    pathChosen = reshape(pathChosen,[lenX,horizon+1]);
    
    r = round(rand(500,1) * (length(paths)-1) + 1);
    ctrlrOut.options = paths(r,:);
    ctrlrOut.subTimes = config.times(idxs);
    ctrlrOut.chosen = pathChosen;
    
    U = bestU;
end

