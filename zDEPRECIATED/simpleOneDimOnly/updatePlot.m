function config = updatePlot(config,history,subTimes,options,chosen)
    if ~isfield(config,'handles')
        figure(100), clf, hold on
        minmaxY = minmax(config.path);
        range = minmaxY(2) - minmaxY(1);
        yaxis(minmaxY + [-0.3*range 0.3*range])
        config.handles.target   = plot(config.times, config.path,'--k');
        config.handles.options  = plot(subTimes,options','b');
        config.handles.history  = plot(config.times(1:length(history)), history,'r','LineWidth',6);
        config.handles.chosen   = plot(subTimes,chosen(1,:),'g','LineWidth',3);
        drawnow;
    else
        set(config.handles.history,'XData',config.times(1:length(history)),'YData',history);
        if exist('subTimes','var')
            set(config.handles.options,'XData',subTimes,{'YData'},num2cell(options,2));
            set(config.handles.chosen,'XData',subTimes,'YData',chosen(1,:));
        else
            set(config.handles.options,'XData',[],'YData',[]);
            set(config.handles.chosen,'XData',[],'YData',[]);
        end
        drawnow;
    end
end

