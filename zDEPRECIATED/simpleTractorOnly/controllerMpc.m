
classdef controllerMpc
    properties
        config
        lenX
        lenU
        path
        pathDtEstimate
        
        closestPointIdx = 1
        maxHorizon = 5
        thisHorizon = 2
        dtHorizon = 1
        horizonPathIdxInc
        stepsLeft
        inputOptions
        numInputOptions
        inputSeries
        numInputSeries
        thisNumInputSeries
        lastU
    end
    methods
        function obj = setup(obj, config)
            obj.config = config;
            obj.path = config.path;
            obj.lenX = config.lenX;
            obj.lenU = config.lenU;
            obj.lastU = [0 config.v]';

            set = {
%                 [deg2rad(-30) deg2rad(-15) 0 deg2rad(15) deg2rad(30)]
                [deg2rad(-40) deg2rad(-15) 0 deg2rad(15) deg2rad(40)]
%                 [deg2rad(-40) 0 deg2rad(40)]
                [config.v]
            };

            dx = diff(obj.path(:,1));
            dy = diff(obj.path(:,2));
            meanPathStep = sqrt(mean((dx.^2+dy.^2)));
            obj.pathDtEstimate = abs(meanPathStep/config.v);
            
            lenS = length(set);
            len1 = length(set{1});
            len2 = length(set{2});
            obj.numInputOptions = len1 * len2;
            obj.inputOptions = zeros(obj.numInputOptions, lenS);
            o = 1;
            for i = 1:len1
                for j = 1:len2
                    obj.inputOptions(o,1) = set{1}(i);
                    obj.inputOptions(o,2) = set{2}(j);
                    o = o + 1;
                end
            end
            
            obj.numInputSeries = obj.numInputOptions^obj.maxHorizon;
            obj.inputSeries = zeros(obj.numInputSeries, obj.lenU, obj.maxHorizon);
            i = 1;
            for h1 = 1:obj.numInputOptions
                for h2 = 1:obj.numInputOptions
                    for h3 = 1:obj.numInputOptions
                        for h4 = 1:obj.numInputOptions
                            for h5 = 1:obj.numInputOptions
                                obj.inputSeries(i,:,1) = obj.inputOptions(h1,:);
                                obj.inputSeries(i,:,2) = obj.inputOptions(h2,:);
                                obj.inputSeries(i,:,3) = obj.inputOptions(h3,:);
                                obj.inputSeries(i,:,4) = obj.inputOptions(h4,:);
                                obj.inputSeries(i,:,5) = obj.inputOptions(h5,:);
                                i = i + 1;
                            end
                        end
                    end
                end
            end
            
            obj.horizonPathIdxInc = round(abs(obj.dtHorizon/obj.pathDtEstimate));
        end
        
        function [obj,inputs] = filterInputSeries(obj)
            % Crop series if horizon != maxHorizon
            obj.thisNumInputSeries = obj.numInputOptions^obj.thisHorizon;
            inputs1 = obj.inputSeries(1:obj.thisNumInputSeries,:,end-obj.thisHorizon+1:end);
            
            % Filter series based on input rates
            firstU = obj.lastU';
            maxUrate = [deg2rad(42), 0];
            
            seriesToKeep = ones(obj.thisNumInputSeries,1,'logical');
            for s = 1:obj.thisNumInputSeries
                prevU = firstU;
                for h = 1:obj.thisHorizon
                    thisU = inputs1(s,:,h);
                    if any(abs(thisU - prevU) > maxUrate)
                        seriesToKeep(s) = 0;
                        break
                    end
                    prevU = thisU;
                end
            end
            
            obj.thisNumInputSeries = sum(seriesToKeep);
            inputs = inputs1(seriesToKeep == 1,:,:);
        end
        
        function [obj,inputs] = filterInputSeries2(obj)
            % Crop series if horizon != maxHorizon
            obj.thisNumInputSeries = obj.numInputOptions^obj.thisHorizon;
            inputs1 = zeros(obj.thisNumInputSeries,obj.lenU,1+obj.thisHorizon);
            inputs1(:,:,1) = repmat(obj.lastU',[obj.thisNumInputSeries,1]);
            inputs1(:,:,2:end) = obj.inputSeries(1:obj.thisNumInputSeries,:,end-obj.thisHorizon+1:end);
            
            maxRate = [deg2rad(42) 0];
            rates = diff(inputs1,1,3);
            rates = abs(rates);
            maxRatesMat = repmat(maxRate,[obj.thisNumInputSeries,1,obj.thisHorizon]);
            largeRates = rates > maxRatesMat;
            seriesToKeep = ~any(any(largeRates,3),2);
            inputs = inputs1(seriesToKeep,:,2:end); 
            obj.thisNumInputSeries = sum(seriesToKeep);

%             inputs = inputs1; 
        end

        function outputs = lookaheadBamboos(obj, X, inputs)
            
            outputs = zeros(obj.thisNumInputSeries,obj.lenX,obj.thisHorizon+1);
            for s = 1:obj.thisNumInputSeries
                x = X;
                outputs(s,:,1) = X;
                for h = 1:obj.thisHorizon
                    U = inputs(s,:,h)';
                    dX = plantDir(obj.config, x, U);
                    x = x + dX*obj.dtHorizon;
                    outputs(s,:,h+1) = x;
                end
            end
        end
        
        function [inputs, outputs] = lookaheadTree(obj,X)
            outputs = zeros(1,obj.lenX,1);
            outputs(1,:,1) = X;
            inputs = zeros(1,obj.lenU,0);
            for h = 1:obj.thisHorizon
                oldOutputs = outputs;
                oldInputs = inputs;
                numNewTests = size(oldOutputs,1)*obj.numInputOptions;
                newOutputs = zeros(numNewTests,obj.lenX,h+1);
                newInputs  = zeros(numNewTests,obj.lenU,h);

                % Continue each output trajectory
                for t = 1:size(oldOutputs,1)
                    % Get history
                    Xs = oldOutputs(t,:,:);
                    Xs = reshape(Xs,[obj.lenX,h]);
                    lastX = oldOutputs(t,:,end);
                    lastX = reshape(lastX,[obj.lenX,1]);
                    Us = oldInputs(t,:,:);
                    Us = reshape(Us,[obj.lenU,h-1]);

                    % Run each input option
                    for i = 1:obj.numInputOptions
                        U = obj.inputOptions(i,:);
                        U = reshape(U,[obj.lenU,1]);
                        dX = plantDir(obj.config, lastX, U);
                        newX = lastX + dX*obj.dtHorizon;

                        % Save output and input
                        newXs = [Xs newX];
                        newOutputs((t-1)*obj.numInputOptions + i, :, :) = newXs;
                        newInputs((t-1)*obj.numInputOptions + i, :, :) = [Us U];
                    end
                end
                outputs = newOutputs;
                inputs = newInputs;
            end

        end
        
        function targetPath = getTargetTrajectory(obj)
            targetPath = zeros(2,obj.thisHorizon+1);
            for h = 1:obj.thisHorizon+1
                targetPath(:,h) = obj.path(obj.closestPointIdx + (h-1)*obj.horizonPathIdxInc,:);
            end
        end
        
        function [bestX, bestU] = findLowestCost(obj,inputs,outputs,targetPath)
            paths = outputs(:,1:2,:);
            numPaths = size(paths,1);
            pathErrors = zeros(length(paths),1);
            for p = 1:numPaths
                pathError = 0;
                for h = 2:obj.thisHorizon+1
                    x = paths(p,1,h);
                    y = paths(p,2,h);
                    xt = targetPath(1,h);
                    yt = targetPath(2,h);
                    tdis = (x-xt)^2 + (y-yt)^2;
%                     [tdis, ~] = findClosestPoint(obj.path, [xt;yt], obj.closestPointIdx);
                    pathError = pathError + tdis;
                end
                pathErrors(p) = pathError;
            end


            [~,minErrIdx] = min(pathErrors);
            bestU = inputs(minErrIdx,:,1);
            bestU = reshape(bestU,[obj.lenU,1]);
            bestX = outputs(minErrIdx,:,:);
            bestX = reshape(bestX,[obj.lenX,obj.thisHorizon+1]);
        end
        
        function [obj,U,ctrlrOut] = loop(obj,X,t)
            %% Find spot on path
            obj.closestPointIdx = findClosestPoint2(obj.path,X,obj.closestPointIdx);

            obj.stepsLeft = floor((length(obj.path)-obj.closestPointIdx)/obj.horizonPathIdxInc);
            obj.thisHorizon = min(obj.maxHorizon,obj.stepsLeft); % Make sure horizon doesn't go past end of path
            obj.thisHorizon = max(obj.thisHorizon, 0);


            %% Simulate
            % 4.6 vs 0.28s for findLowestCost
%             [inputs, outputs] = lookaheadTree(obj, X);
            
            % 6.9s vs 0.23s for findLowestCost
%             [inputs, outputs] = lookaheadBamboos(obj, X);

            [obj,inputs] = filterInputSeries2(obj);
            outputs = lookaheadBamboos(obj, X, inputs);

            %% Run cost function
            targetPath = getTargetTrajectory(obj);
            [bestX, bestU] = findLowestCost(obj, inputs, outputs, targetPath);
            
            %% Format output
            U = bestU;
            obj.lastU = U;
            ctrlrOut.chosen = bestX;
            ctrlrOut.closestPointIdx = obj.closestPointIdx;
            ctrlrOut.options = outputs;
        end
    end
end
