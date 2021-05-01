classdef controllerMpcOdePrecalc
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
            obj.lastU = zeros(config.lenU,1);
            obj.thisHorizon = obj.maxHorizon;

            set = {
%                 [deg2rad(-30) deg2rad(-15) 0 deg2rad(15) deg2rad(30)]
                [deg2rad(-40) deg2rad(-20) deg2rad(-10) 0 deg2rad(10) deg2rad(20) deg2rad(40)]
%                 [deg2rad(-40) 0 deg2rad(40)]
                [0];
            };
            obj.maxRate = [deg2rad(45) deg2rad(45)];

            dx = diff(obj.path(:,1));
            dy = diff(obj.path(:,2));
            meanPathStep = sqrt(mean((dx.^2+dy.^2)));
            obj.pathDtEstimate = abs(meanPathStep/config.v);
            obj.horizonPathIdxInc = round(abs(obj.dtHorizon/obj.pathDtEstimate));

%             lenS = length(set);
            len1 = length(set{1});
%             len2 = length(set{2});
%             obj.numInputOptions = len1 * len2;
            obj.numInputOptions = len1;
            obj.inputOptions = zeros(obj.numInputOptions, obj.lenU);
            o = 1;
            for i = 1:len1
%                 for j = 1:len2
                    obj.inputOptions(o,1) = set{1}(i);
%                     obj.inputOptions(o,2) = set{2}(j);
                    o = o + 1;
%                 end
            end
            
            obj.outputOptions = zeros(obj.numInputOptions, obj.lenX);
            datumX = zeros(config.lenX,1);
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
        
        function Xouts = transformPath(obj,dXs,X)
            x = X(1);
            y = X(2);
            theta = X(3);
            phi = X(4);
            
            Xouts(1,:) = x + dXs(1,:)*cos(theta) - dXs(2,:)*sin(theta);
            Xouts(2,:) = y + dXs(1,:)*sin(theta) + dXs(2,:)*cos(theta);
            Xouts(3,:) = theta + dXs(3,:);
            Xouts(4,:) = phi + dXs(4,:);
        end
                
        function [obj,inputsToUse] = filterInputSeries(obj)
            % Crop series if horizon != maxHorizon
%             obj.thisNumInputSeries = obj.numInputOptions^obj.thisHorizon;
%             inputs1 = zeros(obj.thisNumInputSeries,obj.lenU,1+obj.thisHorizon);
%             inputs1(:,:,1) = repmat(obj.lastU',[obj.thisNumInputSeries,1]);
%             inputs1(:,:,2:end) = obj.inputSeries(1:obj.thisNumInputSeries,:,end-obj.thisHorizon+1:end);
            
            % Don' use series if first input in series is too far from last
            % input that was used.
            rates = squeeze(obj.inputSeries(:,:,1)) - repmat(obj.lastU',[obj.numInputSeries,1]);
            rates = abs(rates);
            maxRatesMat = repmat(obj.maxRate,[obj.numInputSeries,1,1]);
            largeRates = rates > maxRatesMat;
            inputsToUse = ~any(largeRates,2);
            obj.thisNumSeries = sum(inputsToUse);
        end

        function outputs = calculatePaths(obj, X, inputsToUse)
            thisOutputSeries = obj.outputSeries(inputsToUse,:,:);
            outputs = zeros(obj.thisNumSeries,obj.lenX,obj.thisHorizon+1);
            for s = 1:obj.thisNumSeries
                relativeOutput = thisOutputSeries(s,:,1:obj.thisHorizon+1);
                relativeOutput = reshape(relativeOutput,[obj.lenX,obj.thisHorizon+1]);
                outputs(s,:,:) = transformPath(obj,relativeOutput,X);
            end
        end
                
        function targetPath = getTargetTrajectory(obj)
            targetPath = zeros(2,obj.thisHorizon+1);
            for h = 1:obj.thisHorizon+1
                targetPath(:,h) = obj.path(obj.closestPointIdx + (h-1)*obj.horizonPathIdxInc,:);
            end
        end
        
        function [bestX, bestU] = findLowestCost(obj,inputsToUse,outputs,targetPath)
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
            seriesIdx = find(cumsum(inputsToUse) == minErrIdx,1,'first');
            bestU = obj.inputSeries(seriesIdx,:,1);
            bestU = reshape(bestU,[obj.lenU,1]);
            bestX = outputs(minErrIdx,:,:);
            bestX = reshape(bestX,[obj.lenX,obj.thisHorizon+1]);
        end
        
        function [obj,U,ctrlrOut] = loop(obj,X,t)
            obj.closestPointIdx = findClosestPoint2(obj.path,X,obj.closestPointIdx);

            obj.stepsLeft = floor((length(obj.path)-obj.closestPointIdx)/obj.horizonPathIdxInc);
            obj.thisHorizon = min(obj.maxHorizon,obj.stepsLeft); % Make sure horizon doesn't go past end of path
            obj.thisHorizon = max(obj.thisHorizon, 0);


            [obj,inputsToUse] = filterInputSeries(obj);
            outputs = calculatePaths(obj, X, inputsToUse);

            targetPath = getTargetTrajectory(obj);
            [bestX, bestU] = findLowestCost(obj, inputsToUse, outputs, targetPath);
            
            U = bestU;
            obj.lastU = U;
            ctrlrOut.chosen = bestX;
            ctrlrOut.closestPointIdx = obj.closestPointIdx;
            ctrlrOut.options = outputs;
        end
    end
end
