function [U,ctrlrOut] = steerMPCseries(config, initX, thisF)
    % Constants
    horizon = config.horizon;
    horizon = min(horizon,length(config.path)-thisF);
    inputs = config.inputSeqs;
    numSeries = size(inputs,1);
    lenX = config.lenX;
    lenU = config.lenU;
        
    % Create new variables
    outputs = zeros(numSeries,lenX,horizon);
    
    % Simulate
    for s = 1:numSeries
        X = initX;
        for f = 1:horizon
            U = inputs(s,:,f);
            dX = plant(config, X, U);
            X = X + dX*config.dt;
            outputs(s,:,f) = X;
        end
    end
    
    % Run cost function
    idxs = thisF+1:thisF+horizon;
    targetPath = config.path(idxs);
    paths = outputs(:,1,:);
    paths = reshape(paths,[size(outputs,1),horizon]);
    pathErrors = abs(paths - targetPath);
    cumErrors = sum(pathErrors,2);
    [~,minErrIdx] = min(cumErrors);
    chosenInputs = inputs(minErrIdx,:,:);
    chosenInputs = reshape(chosenInputs,[lenU,horizon]);
    chosenInput = inputs(minErrIdx,:,1);
    chosenInput = reshape(chosenInput,[lenU,1]);
    chosenOutputs = outputs(minErrIdx,:,:);
    chosenOutputs = reshape(chosenOutputs,[lenX,horizon]);
    
    r = round(rand(500,1) * (length(paths)-1) + 1);
    ctrlrOut.options = paths(r,:);
    ctrlrOut.subTimes = config.times(idxs);
    ctrlrOut.chosenInput = chosenOutputs;
    ctrlrOut.chosenOutput = chosenOutputs;
    
    U = chosenInput;
end

