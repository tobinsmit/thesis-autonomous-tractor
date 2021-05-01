function [closestPointDis, closestPointIdx] = findClosestPoint(config, X, lastClosestPointIdx)
    path = config.path;
    pathLength = length(path);
    x = X(1);
    y = X(2);
%     t = tic();
%     for i = 1:500000
        % Avg = 1.2s
%         disSquared = (path(:,1) - x).^2 + (path(:,2) - y).^2;
%         [~, closestPointIdx] = min(disSquared);

        % Avg = 1.7s
%         first = max(1,lastClosestPointIdx-100);
%         last = min(length(path),lastClosestPointIdx+100);
%         section = path([first:last],:);
%         disSquared = (section(:,1) - x).^2 + (section(:,2) - y).^2;
%         [~, closestPointIdx] = min(disSquared);
%         closestPointIdx = closestPointIdx + lastClosestPointIdx - 100 -1;


        % Avg = 0.08s
%         closestPointIdx = lastClosestPointIdx;
%         closestPointDis = (path(closestPointIdx,1) - x).^2 + (path(closestPointIdx,2) - y).^2;
% 
%         nextIdx = closestPointIdx + 1;
%         if nextIdx < pathLength
%             nextDis = (path(nextIdx,1) - x).^2 + (path(nextIdx,2) - y).^2;
%             while nextDis < closestPointDis
%                 closestPointIdx = nextIdx;
%                 closestPointDis = nextDis;
%                 nextIdx = closestPointIdx + 1;
%                 if nextIdx > pathLength; break; end
%                 nextDis = (path(nextIdx,1) - x).^2 + (path(nextIdx,2) - y).^2;
%             end
%         end
%         
%         nextIdx = closestPointIdx - 1;
%         if nextIdx > 1
%             nextDis = (path(nextIdx,1) - x).^2 + (path(nextIdx,2) - y).^2;
%             while nextDis < closestPointDis
%                 closestPointIdx = nextIdx;
%                 closestPointDis = nextDis;
%                 nextIdx = closestPointIdx - 1;
%                 if nextIdx < 1; break; end
%                 nextDis = (path(nextIdx,1) - x).^2 + (path(nextIdx,2) - y).^2;
%             end
%         end
        
        % Avg = 0.08s
        closestPointIdx = lastClosestPointIdx;
        closestPointDis = (path(closestPointIdx,1) - x).^2 + (path(closestPointIdx,2) - y).^2;
        idx = lastClosestPointIdx + 1;
        while idx < pathLength
            dis = (path(idx,1) - x).^2 + (path(idx,2) - y).^2;
            if dis < closestPointDis
                closestPointIdx = idx;
                closestPointDis = dis;
            else
                break;
            end
            idx = idx + 1;
        end
        idx = lastClosestPointIdx - 1;
        while idx >= 1
            dis = (path(idx,1) - x).^2 + (path(idx,2) - y).^2;
            if dis < closestPointDis
                closestPointIdx = idx;
                closestPointDis = dis;
            else
                break
            end
            idx = idx - 1;
        end
        
%     end
%     disp([num2str(closestPointIdx), '/', num2str(length(path))])
%     toc(t)
    closestPointDis = sqrt(closestPointDis);
end
