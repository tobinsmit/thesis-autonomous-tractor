classdef controllerMpc3odePrecalc
    properties
        % System config
        config
        path
        des
        lenX
        lenU
        pathDtEstimate
        
        % Controller config
        closestPointIdx = 1
        maxHorizon = 5
        dtHorizon = 1

        % Delta t config
        deltaTOptions
        deltaTMaxRate
        deltaTOptionOutputs
        numDeltaTOptions
        deltaTSeries
        deltaTSeriesIdxs
        deltaTSeriesOutputs
        numDeltaTSeries
        
        % Delta i config
        deltaIOptions
        deltaIMaxRate
        numDeltaIOptions
        deltaISeries
        deltaISeriesIdxs
        deltaISeriesOutputs
        numDeltaISeries

        horizonPathIdxInc
        stepsLeft
        lastU
        thisHorizon
    end
    methods
        function obj = setup(obj, config)
            obj.config = config;
            obj.path = config.path;
            obj.des = config.des;
            obj.lenX = config.lenX;
            obj.lenU = config.lenU;
            obj.lastU = zeros(config.lenU,1);
            obj.thisHorizon = obj.maxHorizon;

%             obj.deltaTOptions = [deg2rad(-40) deg2rad(-30) deg2rad(-20) deg2rad(-10) 0 deg2rad(10) deg2rad(20) deg2rad(30) deg2rad(40)];
            obj.deltaTOptions = [deg2rad(-40) deg2rad(-20) deg2rad(-10) 0 deg2rad(10) deg2rad(20) deg2rad(40)];
%             obj.deltaIOptions = [deg2rad(-40) deg2rad(-20) deg2rad(-10) 0 deg2rad(10) deg2rad(20) deg2rad(40)];
            obj.deltaIOptions = [deg2rad(-40) deg2rad(-20) deg2rad(-5) 0 deg2rad(5) deg2rad(20) deg2rad(40)];
            obj.deltaTMaxRate = deg2rad(45);
            obj.deltaIMaxRate = deg2rad(45);

            obj.numDeltaTOptions = length(obj.deltaTOptions);
            obj.numDeltaIOptions = length(obj.deltaIOptions);

            % Calculate pathDtEstimate and horizonPathIdxInc
            dx = diff(obj.path(:,1));
            dy = diff(obj.path(:,2));
            meanPathStep = sqrt(mean((dx.^2+dy.^2)));
            obj.pathDtEstimate = abs(meanPathStep/config.v);
            obj.horizonPathIdxInc = round(abs(obj.dtHorizon/obj.pathDtEstimate));

            % Calculate deltaTOptionOutputs. This stores the output for each input option
            obj.deltaTOptionOutputs = zeros(obj.numDeltaTOptions, obj.lenX);
            zeroX = zeros(config.lenX,1);
            for o = 1:obj.numDeltaTOptions
                U = [obj.deltaTOptions(o); 0];
                getDx = @(t,X) plantDir(config,X,U);
                [~,Xpath] = ode45(getDx,[0 obj.dtHorizon],zeroX);
                obj.deltaTOptionOutputs(o,:) = Xpath(end,:)';
            end

            % Make delta t series
            obj.numDeltaTSeries = obj.numDeltaTOptions^obj.maxHorizon;
            obj.deltaTSeriesIdxs = zeros(obj.numDeltaTSeries, obj.maxHorizon);
            obj.deltaTSeries = zeros(obj.numDeltaTSeries, obj.maxHorizon);
            o = obj.numDeltaIOptions;
            idxs = 0:obj.numDeltaTSeries-1;
            for h = 1:obj.maxHorizon
                obj.deltaTSeriesIdxs(:,h) = mod(floor((idxs)/o^(obj.maxHorizon-h)),o)+1;
            end
            obj.deltaTSeries = obj.deltaTOptions(obj.deltaTSeriesIdxs);
            
            
            % Make delta i series
            obj.numDeltaISeries = obj.numDeltaIOptions^obj.maxHorizon;
            obj.deltaISeriesIdxs = zeros(obj.numDeltaISeries, obj.maxHorizon);
            obj.deltaISeries = zeros(obj.numDeltaISeries, obj.maxHorizon);
            o = obj.numDeltaIOptions;
            idxs = 0:obj.numDeltaISeries-1;
            for h = 1:obj.maxHorizon
                hc = obj.maxHorizon - h;
                obj.deltaISeriesIdxs(:,h) = mod(floor((idxs)/o^hc),o)+1;
            end
            obj.deltaISeries = obj.deltaIOptions(obj.deltaISeriesIdxs);
                                    
            % Filter delta t series based on rate of change of input
            rates = diff(obj.deltaTSeries,1,2); % 1st derivative, dimension 2
            rates = abs(rates);
            largeRates = rates > obj.deltaTMaxRate;
            seriesToKeep = ~any(largeRates,2);
            obj.deltaTSeries = obj.deltaTSeries(seriesToKeep,:,:);
            obj.deltaTSeriesIdxs = obj.deltaTSeriesIdxs(seriesToKeep,:,:);
            obj.numDeltaTSeries = sum(seriesToKeep);
                        
            % Filter delta i series based on rate of change of input
            rates = diff(obj.deltaISeries,1,2); % 1st derivative, dimension 2
            rates = abs(rates);
            largeRates = rates > obj.deltaIMaxRate;
            seriesToKeep = ~any(largeRates,2);
            obj.deltaISeries = obj.deltaISeries(seriesToKeep,:,:);
            obj.deltaISeriesIdxs = obj.deltaISeriesIdxs(seriesToKeep,:,:);
            obj.numDeltaISeries = sum(seriesToKeep);
                        
            % Calculate delta t output series
            obj.deltaTSeriesOutputs = zeros(obj.numDeltaTSeries, obj.lenX, obj.maxHorizon+1);
%             obj.deltaTSeriesOutputs(:,:,1) = repmat(zeroX',[obj.numDeltaTSeries,1]);
            for s = 1:obj.numDeltaTSeries
                for h = 1:obj.maxHorizon
                    initX = obj.deltaTSeriesOutputs(s,:,h);
                    inputOptionIdx = obj.deltaTSeriesIdxs(s,h);
                    output = obj.deltaTOptionOutputs(inputOptionIdx,:)';
                    newX = transformPath(obj,output,initX);
                    obj.deltaTSeriesOutputs(s,:,h+1) = newX';
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
            phi = X(4);
            
            Xouts(1,:) = x + dXs(1,:)*cos(theta) - dXs(2,:)*sin(theta);
            Xouts(2,:) = y + dXs(1,:)*sin(theta) + dXs(2,:)*cos(theta);
            Xouts(3,:) = theta + dXs(3,:);
            Xouts(4,:) = phi + dXs(4,:);
        end
                
        function [obj,seriesIsValid,numValidSeries] = filterDeltaTSeriesIdxs(obj)
            % Don' use series if first input in series is too far from last
            % input that was used.
            rates = obj.deltaTSeries(:,1) - obj.lastU(1);
            rates = abs(rates);
            largeRates = rates > obj.deltaTMaxRate;
            seriesIsValid = ~any(largeRates,2);
            numValidSeries = sum(seriesIsValid);
        end
        
        function [obj,seriesIsValid,numValidSeries,thisSeriesToTotalSeries] = filterDeltaISeriesIdxs(obj)
            % Don' use series if first input in series is too far from last
            % input that was used.
            rates = obj.deltaISeries(:,1) - obj.lastU(2);
            rates = abs(rates);
            largeRates = rates > obj.deltaTMaxRate;
            seriesIsValid = ~any(largeRates,2);
            numValidSeries = sum(seriesIsValid);
            thisSeriesToTotalSeries = find(seriesIsValid);
        end

        function outputs = calculateDeltaTPaths(obj, X, seriesIsValid, numValidSeries)
            outputSteps = obj.deltaTSeriesOutputs(seriesIsValid,:,:);
            outputs = zeros(numValidSeries,obj.lenX,obj.thisHorizon+1);
            for s = 1:numValidSeries
                steps = outputSteps(s,:,1:obj.thisHorizon+1);
                steps = reshape(steps,[obj.lenX,obj.thisHorizon+1]);
                outputs(s,:,:) = transformPath(obj,steps,X);
            end
        end
                        
        function outputs = calculateDeltaIPathsBamboos(obj, X, ~, numValidSeries, thisSeriesToTotalSeries, delta_ts)
            outputs = zeros(numValidSeries,obj.lenX,obj.thisHorizon+1);
            for s = 1:numValidSeries
                x = X;
                outputs(s,:,1) = x;
                for h = 1:obj.thisHorizon
                    seriesIdx = thisSeriesToTotalSeries(s);
                    delta_i = obj.deltaISeries(seriesIdx,h);
                    U = [delta_ts(h);delta_i];
                    dX = plantDir(obj.config, x, U);
                    x = x + dX*obj.dtHorizon;
                    outputs(s,:,h+1) = x;
                end
            end
        end
        function targetPath = getTargetTrajectory(obj)
            targetPath = zeros(3,obj.thisHorizon+1);
            
            for h = 1:obj.thisHorizon+1
                idx = obj.closestPointIdx + (h-1)*obj.horizonPathIdxInc;
                targetPath(1,h) = obj.des.x(idx);
                targetPath(2,h) = obj.des.y(idx);
                targetPath(3,h) = obj.des.phi(idx);
            end
        end
        
        function [bestTrail, bestDeltaT,minErr] = findBestDeltaT(obj,seriesIsValid,outputs,targetPath)
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
            seriesIdx = find(cumsum(seriesIsValid) == minErrIdx,1,'first');
            bestDeltaT = obj.deltaTSeries(seriesIdx,:);
            bestTrail = outputs(minErrIdx,:,:);
            bestTrail = reshape(bestTrail,[obj.lenX,obj.thisHorizon+1]);
        end
        
        function delta_i = findBestDeltaI(obj, seriesIsValid, numValidSeries, outputs, targetPath)
            pathPhis = outputs(:,4,:);
            pathPhis = reshape(pathPhis,[numValidSeries obj.thisHorizon+1]);
            pathErrors = zeros(numValidSeries,1);
            parfor p = 1:numValidSeries
                pathError = 0;
                for h = 2:obj.thisHorizon+1
                    phi = pathPhis(p,h);
                    phi_des = targetPath(3,h);
                    e_phi = abs(phi - phi_des);
                    pathError = pathError + e_phi;
                end
                pathErrors(p) = pathError;
            end

            [~,minErrIdx] = min(pathErrors);
            seriesIdx = find(cumsum(seriesIsValid) == minErrIdx,1,'first');
            delta_i = obj.deltaISeries(seriesIdx,1);
%             bestTrail = outputs(minErrIdx,:,:);
%             bestTrail = reshape(bestTrail,[obj.lenX,obj.thisHorizon+1]);
        end
        
        function [obj,U,ctrlrOut] = loop(obj,X,t)
            obj.closestPointIdx = findClosestPoint(obj.path,X,obj.closestPointIdx);

            obj.stepsLeft = floor((length(obj.path)-obj.closestPointIdx)/obj.horizonPathIdxInc);
            obj.thisHorizon = min(obj.maxHorizon,obj.stepsLeft); % Make sure horizon doesn't go past end of path
            obj.thisHorizon = max(obj.thisHorizon, 0);

            targetPath = getTargetTrajectory(obj);

            % Tractor
            [obj,seriesIsValid,numValidSeries] = filterDeltaTSeriesIdxs(obj);
            deltaToutputs = calculateDeltaTPaths(obj, X, seriesIsValid, numValidSeries);
            [delta_t_trail, delta_ts] = findBestDeltaT(obj, seriesIsValid, deltaToutputs, targetPath);
            
            % Implement            
            [obj,seriesIsValid,numValidSeries,thisSeriesToTotalSeries] = filterDeltaISeriesIdxs(obj);
            deltaIoutputs = calculateDeltaIPathsBamboos(obj, X, seriesIsValid, numValidSeries, thisSeriesToTotalSeries, delta_ts);

            delta_i = findBestDeltaI(obj, seriesIsValid, numValidSeries, deltaIoutputs, targetPath);            
            
            U = [delta_ts(1); delta_i];
            obj.lastU = U;
%             ctrlrOut.chosen = bestX;
            ctrlrOut.closestPointIdx = obj.closestPointIdx;
            ctrlrOut.minErr = obj.closestPointIdx;
            ctrlrOut.trac_options = deltaToutputs;
            ctrlrOut.trac_chosen = delta_t_trail;
%             ctrlrOut.options = outputs;
        end
    end
end
