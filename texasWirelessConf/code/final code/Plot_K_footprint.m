function Plot_K_footprint(sensors,r_footprint)
%
% \begin{equation}
% K_{footprint} = \frac{2}{N^2-N}  \sum_{i=1}^N \sum_{j=i+1}^N \left(  \left\| p_i - p_j \right\| _2  \le r_{footprint} \right).
% \end{equation}
if nargin <1
sensors=[0,0;-2,2;2,2;-2,-2;2,-2;-6,6;-8,8;-4,8;-8,4;-4,4;6,-6;8,-8;4,-8;8,-4;4,-4;-10,-10;10,10;-8,-10;-10,-8;8,10;10,8];%Intresting points
r_footprint = 3;
end

D = pdist(sensors);  %pairwise distances

dval = sort(D);
n = numel(dval);

figure(1)
clf
stairs([0,dval],(0:n)/n,'linewidth',2)
xlabel('\it r_{footprint}')
ylabel('\it K_{footprint}')
set(gca,'Ytick',[0,.5,1])
% figure(2)
% clf
axes('Position',[0.55,0.35,0.4,0.4])
%make out interesting function
szInteresting = 200;
phi = zeros(szInteresting,szInteresting);
%these are interesting!!!
[mX,mY] = meshgrid(linspace(-11,11,szInteresting),linspace(-11,11,szInteresting)); %making the grid
for m = 1:size(sensors,1)
    h = 1.25;
    phi=phi+TransmissionCostCalc(h,sensors(m,1), sensors(m,2),6, mX, mY,r_footprint);
end
imagesc(mX(1,:),mY(:,1),phi);
hold on
    set(gca,'YDir','normal');
    %colormap lines
    colormap copper
plot( sensors(:,1),sensors(:,2), 'o','LineWidth',2)
set(gca,'Xtick',[-10,0,10],'Ytick',[-10,0,10])
axis equal
axis tight
title(['\it r_{footprint} = ',num2str(r_footprint)])

set(gcf,'papersize',[7,4])
set(gcf,'paperposition',[0,0,7,4])
print -dpdf '../../pictures/Kfootprint.pdf'

end

function [O] = TransmissionCostCalc(h, xs, ys, pow, xr, yr,r_footprint) %#ok<INUSL>
%  h is height of robot
% [xs,ys] center of sensor nodes,
% [xr,yr] are grid points
%Defining a function for the intresting points
% We would use it in waypointControlForPathPlaning as a function that
% generates intresting points for robots
%O = 1./(((h)^(2))+((xs-xr).^(2))+((ys-yr).^(2))).^((pow)/(2));
O =   1* ( ((xs-xr).^2+(ys-yr).^2)<r_footprint); %Binary cost -- is it better?
end

