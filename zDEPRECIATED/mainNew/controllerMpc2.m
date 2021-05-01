classdef controllerMpc2
    properties
        % System config
        config
        lenX
        lenU
        path
        pathDtEstimate
        
        % Controller config
        closestPointIdx = 1
        maxHorizon = 5
        dtHorizon = 1
        maxRate
        
        % Controller constants
        horizonPathIdxInc
        
        inputOptions
        numInputOptions
        outputOptions
        
        inputSeries
        inputSeriesIdxs
        numInputSeries
        outputSeries

        % Controller variables
        stepsLeft
        lastU
        thisHorizon
        thisNumSeries
    end
    methods
        function obj = setup(obj, config)
            obj.config = config;
            obj.path = config.path;
            obj.lenX = config.lenX;
            obj.lenU = config.lenU;
            obj.lastU = 0;
            obj.thisHorizon = obj.maxHorizon;

            set = {
%                 [deg2rad(-40) deg2rad(-20) deg2rad(-10) 0 deg2rad(10) deg2rad(20) deg2rad(40)]
                [deg2rad(-40) deg2rad(-10) 0 deg2rad(10) deg2rad(40)]
            };
            obj.maxRate = [deg2rad(42)];

            dx = diff(obj.path(:,1));
            dy = diff(obj.path(:,2));
            meanPathStep = sqrt(mean((dx.^2+dy.^2)));
            obj.pathDtEstimate = abs(meanPathStep/config.v);
            obj.horizonPathIdxInc = round(abs(obj.dtHorizon/obj.pathDtEstimate));

            len1 = length(set{1});
            obj.numInputOptions = len1;
            obj.inputOptions = zeros(obj.numInputOptions, obj.lenU);
            o = 1;
            for i = 1:len1
                    obj.inputOptions(o,1) = set{1}(i);
%                     obj.inputOptions(o,2) = set{2}(j);
                    o = o + 1;
%                 end
            end
            
            obj.outputOptions = zeros(obj.numInputOptions, obj.lenX);
            datumX = [0;0;0;0];
            for o = 1:obj.numInputOptions
                U = obj.inputOptions(o,:)';
                getDx = @(t,X) plantDir(config,X,U);
                [~,Xpath] = ode45(getDx,[0 obj.dtHorizon],datumX);
                obj.outputOptions(o,:) = Xpath(end,:)';
            end

            
            obj.numInputSeries = obj.numInputOptions^obj.maxHorizon;
            obj.inputSeries = zeros(obj.numInputSeries, obj.lenU, obj.maxHorizon);
            obj.inputSeriesIdxs = zeros(obj.numInputSeries, obj.maxHorizon);
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
                                obj.inputSeriesIdxs(i,1) = h1;
                                obj.inputSeriesIdxs(i,2) = h2;
                                obj.inputSeriesIdxs(i,3) = h3;
                                obj.inputSeriesIdxs(i,4) = h4;
                                obj.inputSeriesIdxs(i,5) = h5;
                                i = i + 1;
                            end
                        end
                    end
                end
            end
                        
            % Filter inputSeries
            rates = diff(obj.inputSeries,1,3);
            rates = abs(rates);
            maxRatesMat = repmat(obj.maxRate,[obj.numInputSeries,1,obj.thisHorizon-1]);
            largeRates = rates > maxRatesMat;
            seriesToKeep = ~any(any(largeRates,3),2);
            obj.inputSeries = obj.inputSeries(seriesToKeep,:,:);
            obj.inputSeriesIdxs = obj.inputSeriesIdxs(seriesToKeep,:,:);
            obj.numInputSeries = sum(seriesToKeep);
                        
            obj.outputSeries = zeros(obj.numInputSeries, obj.lenX, obj.maxHorizon+1);
            obj.outputSeries(:,:,1) = repmat(datumX',[obj.numInputSeries,1]);
            for s = 1:obj.numInputSeries
                for h = 1:obj.maxHorizon
                    initX = obj.outputSeries(s,:,h);
                    inputOptionIdx = obj.inputSeriesIdxs(s,h);
                    output = obj.outputOptions(inputOptionIdx,:)';
                    newX = transformPath(obj,output,initX);
                    obj.outputSeries(s,:,h+1) = newX';
               end
            end
            
            
        end
        
        function res = canContinue(obj)
            res = and(obj.closestPointIdx < obj.config.pathLen-1, obj.thisHorizon > 1);
        end
        
        function Xouts = transformPath(obj,dXs,X)
            x = X(1);
            y = X(2);
            theta = X(3);
                        
            Xouts(1,:) = x + dXs(1,:)*cos(theta) - dXs(2,:)*sin(theta);
            Xouts(2,:) = y + dXs(1,:)*sin(theta) + dXs(2,:)*cos(theta);
            Xouts(3,:) = dXs(3,:) + theta;
            Xouts(4,:) = X(4);
        end
                
        function [obj,inputs] = filterDeltaTSeries(obj)
            % Crop series if horizon != maxHorizon
            if obj.thisHorizon ~= obj.maxHorizon
                obj.thisNumSeries = find(obj.inputSeries(:,:,1) == obj.inputOptions(1,:),1,'last');
            else
                obj.thisNumSeries = obj.numInputSeries;
            end
            inputs = zeros(obj.thisNumSeries,obj.lenU,1+obj.thisHorizon);
            inputs(:,:,1) = repmat(obj.lastU',[obj.thisNumSeries,1]);
            inputs(:,:,2:end) = obj.inputSeries(1:obj.thisNumSeries,:,end-obj.thisHorizon+1:end);
            % Don' use series if first input in series is too far from last
            % input that was used.
            rates = inputs(:,:,1) - repmat(obj.lastU',[obj.thisNumSeries,1]);
            rates = abs(rates);
            maxRatesMat = repmat(obj.maxRate,[obj.numInputSeries,1,1]);
            largeRates = rates > maxRatesMat;
            inputsToUse = ~any(largeRates,2);
            inputs = inputs(inputsToUse,:,:);
            obj.thisNumSeries = sum(inputsToUse);
        end

        function outputs = lookaheadBamboo(obj, X, inputs)
            outputs = zeros(obj.thisNumSeries,obj.lenX,obj.thisHorizon+1);
            for s = 1:obj.thisNumSeries                
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
        
        function outputs = lookaheadTreeNew(obj,X,inputs)
            outputs = zeros(obj.thisNumSeries,obj.lenX,obj.thisHorizon+1);
            for h = 1:obj.thisHorizon
                
            end
        end

        function outputs = lookaheadTreeOld(obj,X,~)
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

%         function targetPath = getTargetTrajectory(obj)
%             targetPath = zeros(2,obj.thisHorizon+1);
%             for h = 1:obj.thisHorizon+1
%                 targetPath(:,h) = obj.path(obj.closestPointIdx + (h-1)*obj.horizonPathIdxInc,:);
%             end
%         end
        
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
            obj.closestPointIdx = findClosestPoint(obj.path,X,obj.closestPointIdx);

            obj.stepsLeft = floor((length(obj.path)-obj.closestPointIdx)/obj.horizonPathIdxInc);
            obj.thisHorizon = min(obj.maxHorizon,obj.stepsLeft); % Make sure horizon doesn't go past end of path
            obj.thisHorizon = max(obj.thisHorizon, 0);


            [obj,inputs] = filterDeltaTSeries(obj);
%             outputs = calculatePaths(obj, X, inputs);
            outputs = lookaheadTreeOld(obj, X, inputs);

            targetPath = getTargetTrajectory(obj);
            [bestX, bestU] = findLowestCost(obj, inputs, outputs, targetPath);
            
            U = bestU;
            obj.lastU = U;
            ctrlrOut.chosen = bestX;
            ctrlrOut.closestPointIdx = obj.closestPointIdx;
            ctrlrOut.options = outputs;
        end
    end
end
