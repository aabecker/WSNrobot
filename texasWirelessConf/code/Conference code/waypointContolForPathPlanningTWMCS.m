function costs = waypointContolForPathPlanningTWMCS()
% original code, 5 iterations took 45.536606 seconds.
% new code, 10 iterations took 4.304669 seconds.
% Based on
% algorithm1, @article{SolteroEtAlIJRR14VoronoiPathPlanning,
%   author = {D. E. Soltero and M. Schwager and D. Rus},
%   title = {Decentralized path planning for coverage tasks using gradient descent adaptive control},
%   journal = {International Journal of Robotics Research},
%   month = {March},
%   year = {2014},
%   volume = {33},
%   number = {3},
%   pages = {401--425}}
%
% Srikanth and Aaron T. Becker
% Date 2/11/2015
% setup instructions (call this at the beginning)


%Using the HILBERT CURVE FOR INITIALIZING WAYPOINTS
[x,y] = hilbert(3);
waypointshilbert=20*[x',y'];
waypoints=[0 0;waypointshilbert];
% waypoints=[             0         0
%    -3.1000   -1.8855
%    -4.7682   -0.6228
%    -6.2449    0.8928
%    -7.5992    2.8118
%    -8.8443    3.5269
%    -6.8064    4.0521
%    -5.1893    4.2228
%    -8.4235    4.8959
%    -8.1699    7.0552
%    -7.9124    8.5694
%    -5.6148    7.5145
%    -6.4108    5.8415
%    -4.5995    5.7773
%    -4.3120    8.7031
%    -2.9616    8.2505
%    -3.7348    7.2498
%    -3.1161    4.0071
%    -1.4072    2.7652
%    -3.2762    2.1875
%    -1.6049    1.3033
%    -2.1848   -1.2648
%    -3.3562   -4.4904
%    -5.3745   -7.9449
%    -6.6544   -9.8127
%    -7.6542  -10.1421
%   -10.1241   -9.3487
%    -9.8137   -7.2978
%    -7.5480   -8.9840
%    -4.2765   -6.1808
%    -2.5304   -2.8473
%    -1.3956   -2.7386
%    -0.4632   -1.4528
%     1.6753   -1.4781
%     2.5098   -1.7116
%     1.7798   -2.8678
%     4.6757   -3.4956
%     4.0793   -5.0710
%     3.4855   -6.9408
%     3.1477   -7.6114
%     3.2233   -8.1961
%     6.2618   -8.0358
%     8.4009   -8.4639
%     8.0282   -6.8302
%     6.3025   -5.7362
%     6.9597   -3.9151
%     8.5997   -4.3973
%     7.3879   -2.8633
%     8.1462   -2.9221
%     8.2889   -0.4860
%     8.5177    2.0049
%     8.8440    4.6375
%     9.2525    7.4306
%    10.4305    8.3210
%     9.1105   10.2777
%     7.8699    9.6255
%     7.3258   10.4517
%     6.8333    9.0557
%     4.7760    6.7645
%     3.4005    4.9037
%     2.3788    3.0083
%     1.0671    2.2919
%     2.5886    2.2854
%     2.4028    1.4889
%     1.5214    0.6509];

% waypoints   =[                 0         0
%    -1.8753   -0.9254
%    -1.8963    1.0894
%    -1.3125    2.6236
%    -3.3183    2.3830
%    -3.0896    4.2261
%    -4.0303    7.1301
%    -3.0345    8.1853
%    -4.4165    8.7613
%    -5.7818    7.3051
%    -7.8575    8.6771
%    -8.2540    7.1937
%    -6.5478    5.8889
%    -4.7214    5.6668
%    -5.1958    4.0884
%    -6.6001    4.1332
%    -8.1951    4.9911
%    -9.0710    3.7714
%    -7.6406    3.2162
%    -6.1295    1.5995
%    -4.6118   -0.0064
%    -3.0802   -1.5944
%    -3.5139   -3.9596
%    -5.1432   -6.9831
%    -6.7668  -10.0007
%    -8.4152  -10.1897
%   -10.2923   -9.6149
%    -9.8241   -7.3987
%    -7.4572   -8.7963
%    -5.0178   -5.8324
%    -2.5857   -2.8654
%    -1.1911   -2.7288
%    -0.2430   -1.4144
%     1.2392   -1.5067
%     2.3129   -1.4335
%     1.7984   -2.9537
%     4.5962   -3.4462
%     4.0811   -5.0639
%     3.6115   -6.7332
%     3.1811   -7.6702
%     3.1796   -8.6221
%     6.2705   -8.1050
%     8.4660   -8.6590
%     8.1417   -7.1137
%     6.3242   -5.8960
%     6.6946   -4.3470
%     7.3505   -3.0256
%     8.5280   -4.6665
%     8.6120   -3.0744
%     8.8719   -0.4778
%     9.1795    2.1138
%     9.5591    4.7013
%    10.0085    7.2923
%    10.3897    9.0035
%    10.1055   10.3146
%     8.7534    9.0525
%     7.6258   10.4800
%     7.2643    9.1839
%     5.6020    7.1851
%     3.9211    5.1975
%     2.2152    3.2175
%     2.4518    2.1200
%     2.3067    1.0249
%     1.5651    0.0928
%     0.5761    1.8525];

global MAKE_MOVIE G
G.fig = figure(1);
clf
G.init = false;
MAKE_MOVIE = false;
if MAKE_MOVIE
    MOVIE_NAME = 'WAY POINT CONTROL FOR PATH PLANNING one robot case'; %#ok<*UNRCH>
    
    clf
    set(G.fig,'Units','normalized','outerposition',[0 0 1 1],'NumberTitle','off','MenuBar','none','color','w');
    G.writerObj = VideoWriter(MOVIE_NAME,'MPEG-4');%http://www.mathworks.com/help/matlab/ref/videowriterclass.html
    set(G.writerObj,'Quality',100);
    open(G.writerObj);
end

set(G.fig,'Name','IC path controller');
%make out interesting function
szInteresting = 200;
phi = zeros(szInteresting,szInteresting);
%these are interesting!!!
[mX,mY] = meshgrid(linspace(-11,11,szInteresting),linspace(-11,11,szInteresting)); %making the grid
cellsz = mY(2) - mY(1);
sensors=[0,0;-2,2;2,2;-2,-2;2,-2;-6,6;-8,8;-4,8;-8,4;-4,4;6,-6;8,-8;4,-8;8,-4;4,-4;-10,-10;10,10;-8,-10;-10,-8;8,10;10,8];%Intresting points
rng(10); %seed the random number generator,  rng(10); is nice for sensors = [0,0; 20*(rand(15,2)-.5)];
%sensors = [0,0; 20*(rand(15,2)-.5)];

for m = 1:size(sensors,1)
    h = 1.25;
    phi=phi+TransmissionCostCalc(h,sensors(m,1), sensors(m,2),6, mX, mY);
end

indExciting = find(phi>0); %index of every 'exciting point'
% constants for cost funtions:
Wn=1;
Ws = 1;
Ki=1; %potentially -time varying positive definite matrix

G = plotWaypoints(waypoints,mX,mY,phi,0,G);
nwaypts = size(waypoints,1);

%construct matrix of all data points and distances
indx = indExciting(m);
dists = zeros(numel(indx),nwaypts);
poses = zeros(numel(indx),2);
minDists = zeros(numel(indx),1);
minWPs = zeros(numel(indx),1);
for m = 1:numel(indExciting)  % iterate through every grid point that is 'exciting'
    indx = indExciting(m);
    pos = [mX(indx)+cellsz/2,mY(indx)+cellsz/2];  %center of grid cell is [mX,mY] + 1/2[cellsize,cellsize], pos is the center of the interesting gridcell
    poses(m,:) = pos;
    %squared distance between this grid cell and every waypoint
    sumSqDist = sum((repmat(pos,nwaypts,1) - waypoints).^2,2);
    dists(m,:) = sumSqDist;
    [minDist,minWP] = min(sumSqDist);
    minDists(m) = minDist;
    minWPs(m) = minWP;
end

tic
maxIters = 100;
costs = zeros(maxIters+1,1);
costs(1) = calcCost( minDists,phi,indx,Ws,Wn,waypoints);

for iter = 1:maxIters
    for wp=2:nwaypts %iterate through all waypoints
        smallerInd = dists(:,wp)<=minDists';
        
        M_val = sum(phi(smallerInd)); %calculate the mass
        L_val = sum(poses(smallerInd,:).*repmat(phi(smallerInd),1,2)); %calculate the mass
        
        %calculate errors between centroid and desired position
        if M_val == 0
            e_val=[0,0];
        else
            C_val = L_val./[M_val,M_val];
            e_val= C_val - [waypoints(wp,1),waypoints(wp,2)];
        end
        
        if wp ==nwaypts % be careful to wrap the last waypoint back to the first
            alpha_val=Wn*(waypoints(wp-1,:)+waypoints(1,:)-2*waypoints(wp,:));
        else
            alpha_val=Wn*(waypoints(wp-1,:)+waypoints(wp+1,:)-2*waypoints(wp,:));
        end
        beta_val=M_val+(2*Wn);
        
        % moving towards the centeroid of the voronoi cells
        u=Ki.*((M_val.*e_val)+alpha_val)/beta_val; %Control input based on gradient descent
        if ~isequal(u,[0,0])
            %apply control input
            deltat=0.1; %time period
            waypoints(wp,:)=waypoints(wp,:)+u*deltat; %updating waypoints
            %recalculate if this waypoint moved
            dists(:,wp) = sum( (poses - repmat(waypoints(wp,:),numel(indExciting),1)).^2,2);
            oldMinIndices = find(minWPs == wp);
            for m = oldMinIndices
                if dists(m,wp)<minDists(m)
                    minDists(m) = dists(m,wp);
                else %need to check if another wp is closer
                    [minDist,minWP] = min(dists(m,:));
                    minDists(m) = minDist;
                    minWPs(m) = minWP;
                end
            end
            smallerInd = dists(:,wp)<=minDists';
            minWPs(smallerInd) = wp;  %update if this WP is now the closest
            minDists(smallerInd) = minDists(smallerInd);
        end
        
        %         if ( iter<3 && mod(wp,2)) || mod(wp,10)
        %             plotWaypoints(waypoints,mX,mY,phi,iter) % calling function
        %         end
     axis([-11 11 -11 11])   
    end
%     W=iter+200;
    G = plotWaypoints(waypoints,mX,mY,phi,iter,G); % calling function
    %TODO:  make a flag to decide whether to do ANY plotting
    
    costs(iter+1) = calcCost( minDists,phi,indx,Ws,Wn,waypoints);
    
end
toc
disp(waypoints);
if MAKE_MOVIE
    close(G.writerObj);
end

figure(2)
clf;
% plot(costs)
% xlabel('iteration')
% ylabel('Cost function')
end

function cost = calcCost( minDists,phi,indx,Ws,Wn,waypoints)

cost = ( Ws/2*sum(minDists*phi(indx))...
    + Wn/2*sum( (  ...
     (waypoints(:,1) - [waypoints(2:end,1);waypoints(1,1)]).^2 +... %x-distances
    (waypoints(:,2) - [waypoints(2:end,2);waypoints(1,2)]).^2 ).^1 ...  % ydistances
    ));
end


function G = plotWaypoints(waypoints,mX,mY,phi,iter,G)
%         pcolor(mX,mY,phi)
if ~G.init
    G.hPhiPlot = imagesc(mX(1,:),mY(:,1),phi);
    set(gca,'YDir','normal');
    %colormap lines
    colormap copper
    %         colormap([0,0,0;1,0,0])
    hold on
    G.hPath = line([waypoints(:,1);waypoints(1,1)],[waypoints(:,2);waypoints(1,2)],'color','m');
    set(G.hPath, 'linewidth',2);
    G.hWaypoints = plot(waypoints(:,1),waypoints(:,2),'go');
    axis equal
    axis tight
    a = axis;
    [vx,vy] = voronoi(waypoints(:,1),waypoints(:,2));
    G.hVoronoi = plot(vx,vy,'b-');
    uistack(G.hVoronoi,'bottom');
    axis(a);  % voronoi tends to make the axis zoom out.
    uistack(G.hPhiPlot, 'bottom')
     title 'Intresting function with the binary trasmission cost, P=3';
     G.htitle = title(num2str(iter));
    xlabel 'X-axis(m)'
    ylabel 'Y-axis(m)'
    G.init =true;
else
    set(G.hPhiPlot,'Cdata',phi);
    set(G.hWaypoints,'Xdata',waypoints(:,1),'Ydata',waypoints(:,2));
    set(G.hPath,'Xdata',[waypoints(:,1);waypoints(1,1)],'Ydata',[waypoints(:,2);waypoints(1,2)]);
    set(G.htitle,'string', num2str(iter));
    %computing the waypoint's Voronoi Partition
%     [vx,vy] = voronoi(waypoints(:,1),waypoints(:,2));
%     set(G.hVoronoi,'Xdata',vx,'Ydata',vy)
    
    
    a = axis;
    [vx,vy] = voronoi(waypoints(:,1),waypoints(:,2));
    delete(G.hVoronoi)
    G.hVoronoi = plot(vx,vy,'b-');
    uistack(G.hVoronoi,'bottom');
    axis(a);  % voronoi tends to make the axis zoom out.
    uistack(G.hPhiPlot, 'bottom')
    
    
end
% voronoi(waypoints(:,1),waypoints(:,2));


drawnow
makemovie()

end

function makemovie()
global MAKE_MOVIE G
if MAKE_MOVIE
    % (for each frame)
    figure(G.fig)
    %set(gcf,'renderer','painters')  %use these settings for final
    %tfig = myaa(3);
    F = getframe;
    writeVideo(G.writerObj,F.cdata);
    %close(tfig)
end
end
function [O] = TransmissionCostCalc(h, xs, ys, pow, xr, yr) %#ok<INUSL>
%  h is height of robot
% [xs,ys] center of sensor nodes,
% [xr,yr] are grid points
%Defining a function for the intresting points
% We would use it in waypointControlForPathPlaning as a function that
% generates intresting points for robots
%O = 1./(((h)^(2))+((xs-xr).^(2))+((ys-yr).^(2))).^((pow)/(2));
O =   1* ( ((xs-xr).^2+(ys-yr).^2)<3 ); %Binary cost -- is it better?
end
% set(gcf,'papersize',[6,6]);
% set(gcf,'paperposition',[0,0,6,6]);
% print -dpdf '1_6.pdf'