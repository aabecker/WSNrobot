function Plot_K_footprintVIDEO(sensors)
% Makes movie showing the effect of varying K_footprint
set(0,'defaultaxesfontsize',32);
set(0,'defaulttextfontsize',32);
%
% \begin{equation}
% K_{footprint} = \frac{2}{N^2-N}  \sum_{i=1}^N \sum_{j=i+1}^N \left(  \left\| p_i - p_j \right\| _2  \le r_{footprint} \right).
% \end{equation}
if nargin <1
    sensors=[0,0;-2,2;2,2;-2,-2;2,-2;-6,6;-8,8;-4,8;-8,4;-4,4;6,-6;8,-8;4,-8;8,-4;4,-4;-10,-10;10,10;-8,-10;-10,-8;8,10;10,8];%Intresting points
end


 MOVIE_NAME = 'KfootprintMovie';
  fig = figure(1);
    clf
    set(fig,'Units','normalized','outerposition',[0 0 1 1],'NumberTitle','off','MenuBar','none','color','w');
    writerObj = VideoWriter(MOVIE_NAME,'MPEG-4');%http://www.mathworks.com/help/matlab/ref/videowriterclass.html
    set(writerObj,'Quality',100);
    open(writerObj);

range = [1:0.1:30,30:-1:1];

for r_footprint = range
figure(fig)
clf
D = pdist(sensors);  %pairwise distances
dval = sort(D);
n = numel(dval);

[~,in] = find(dval>=r_footprint,1,'first');
if isempty(in)
    in = n;
end
%0.095258*
plot( r_footprint*[0,1,1],(in-1)/n*[1,1,0],'r','LineWidth',2)
hold on
stairs([0,dval],(0:n)/n,'linewidth',2)
xlabel('\it r_{footprint}')
ylabel('\it K_{footprint}')
set(gca,'Ytick',[0,.5,1])
% figure(2)
% clf
axes('Position',[0.55,0.35,0.4,0.4])
%make out interesting function
szInteresting = 400;
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
    %caxis([0,21])
    colormap copper
    
    th = linspace(0,2*pi,50);
plot(r_footprint*cos(th)-10,r_footprint*sin(th)-10,'r','LineWidth',2);
plot(r_footprint*cos(th),r_footprint*sin(th),'r','LineWidth',2);
plot( sensors(:,1),sensors(:,2), 'o','LineWidth',2)
set(gca,'Xtick',[-10,0,10],'Ytick',[-10,0,10])
axis equal
axis(11*[-1,1,-1,1])


title(['\it r_{footprint} = ',num2str(r_footprint,'%4.1f\n')],  'Units', 'normalized', ...
'Position', [0 1], 'HorizontalAlignment','left' )

%pause(0.5)
           % (for each frame)
            figure(fig)
            %axes('Position',[0,0,1,1])
           
           % set(gcf,'renderer','painters')   %optional line to remove antialiasing 
           % tfig = myaa(3);   %optional line 2
            F = getframe(fig);
            writeVideo(writerObj,F.cdata);
          %  close(tfig)

end

title('finished')
close(writerObj);
end


function [O] = TransmissionCostCalc(h, xs, ys, pow, xr, yr,r_footprint) %#ok<INUSL>
%  h is height of robot
% [xs,ys] center of sensor nodes,
% [xr,yr] are grid points
%Defining a function for the intresting points
% We would use it in waypointControlForPathPlaning as a function that
% generates intresting points for robots
%O = 1./(((h)^(2))+((xs-xr).^(2))+((ys-yr).^(2))).^((pow)/(2));
O =   1* ( ((xs-xr).^2+(ys-yr).^2)<r_footprint^2); %Binary cost -- is it better?
end

