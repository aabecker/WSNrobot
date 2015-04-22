

p1=plot(nan,nan,'s','markersize',16,'markeredgecolor','k','linewidth',2, 'markerfacecolor',[1,0,0]);
hold on
p2=plot(nan,nan,'s','markersize',16,'markeredgecolor','k','linewidth',2, 'markerfacecolor',[0,1,0]);
p3=plot(nan,nan,'-m','linewidth',2);
legend([p1,p2,p3],{'full power','half power','link'},'location','eastOutside')
