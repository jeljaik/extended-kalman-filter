function [ output_args ] = plotFilterResultTimeSeries( t, x, y, P,idx, plotText,cols,subFig )
%PLOTFILTERRESULTTIMESERIES PLotting a time series composed of a state of a
%filtered system. Requires the full state definition and the covariance.
%Can be passed on or several state indices.
    
figure();
numSubFig = length(idx);%size(x,2);
if(isempty(subFig))
    subFig = [numSubFig,1];
    
end

for i = 1:numSubFig
    subplot(subFig(1),subFig(2),i);
    shadedErrorBar(t,x(:,idx(i)),squeeze(2*sqrt(P(idx(i),idx(i),:)))',cols{i}, 1);
    
    if(~isempty(y))
        hold on
        plot(t, y(:,idx(i)), '--k','linewidth',2);
    end
    title(plotText.titleText{i});
    xlabel(plotText.xlabelText{i});
    ylabel(plotText.ylabelText{i});
    set(gca,'FontSize',12);
    a = axis();del_a = a(3)-a(4);
    
    axis([t(1) t(end) a(3)+0.1*del_a a(4)-0.1*del_a]);
    g = get(gca);set(gca,'yTick',linspace(g.YTick(1),g.YTick(end),3));    
end
    
    


end

