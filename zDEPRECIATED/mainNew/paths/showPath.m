for i = 1:9
    load(['path' num2str(i) '.mat'])
    figure(i)
    plot(path(:,1),path(:,2))
    xrange = minmax(path(:,1)');
    yrange = minmax(path(:,2)');
    xlimPlot(1) = xrange(1) - 0.1*diff(xrange);
    xlimPlot(2) = xrange(2) + 0.1*diff(xrange);
    ylimPlot(1) = yrange(1) - 0.1*diff(yrange);
    ylimPlot(2) = yrange(2) + 0.1* diff(yrange);
    ylim(ylimPlot)
    xlim(xlimPlot)
    axis equal
end