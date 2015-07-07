function [ output_args ] = comparisonPlotFilterResultTimeSeries( t, x_nsnc,x_nc,x_c, idx, plotText,cols,subFig )
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
    plot(t,x_nsnc(:,idx(i)),cols{1});
    hold on
    plot(t,x_nc(:,idx(i)),cols{2});
    hold on
    plot(t,x_c(:,idx(i)),cols{3});
%   shadedErrorBar(t,x(:,idx(i)),squeeze(2*sqrt(P(idx(i),idx(i),:)))',cols{i}, 1);
    legend('a','b','c');
    title(plotText.titleText{i});
    xlabel(plotText.xlabelText{i});
    ylabel(plotText.ylabelText{i});
    set(gca,'FontSize',12);
    a = axis();del_a = a(3)-a(4);
    
    axis([t(1) t(end) a(3)+0.1*del_a a(4)-0.1*del_a]);
    g = get(gca);set(gca,'yTick',linspace(g.YTick(1),g.YTick(end),3));    
end
    
    


end

