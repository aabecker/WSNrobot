function TwoRobotWaypointContolForPathPlanningTWMCS()

%Using the HILBERT CURVE FOR INITIALIZING WAYPOINTS

global MAKE_MOVIE G
G.fig = figure(1);
clf
G.init = false;
MAKE_MOVIE = false;
if MAKE_MOVIE
    MOVIE_NAME = 'WAY POINT CONTROL FOR PATH PLANNING'; %#ok<*UNRCH>
    
    clf
    set(G.fig,'Units','normalized','outerposition',[0 0 1 1],'NumberTitle','off','MenuBar','none','color','w');
    G.writerObj = VideoWriter(MOVIE_NAME,'MPEG-4');%http://www.mathworks.com/help/matlab/ref/videowriterclass.html
    set(G.writerObj,'Quality',100);
    open(G.writerObj);
end
waypoints1=[         0         0
    1.5666    0.6141
    2.3415    1.7018
    2.4076    1.2928
    4.5798    3.1806
    6.7422    5.2761
    8.9001    7.6432
   10.3905    8.0013
    9.1954   10.2322
    7.3280   10.4516
    7.8666    9.6395
    6.8351    9.0446
    4.7695    6.7555
    3.3855    4.8906
    2.3498    2.9911
    2.5978    2.3467
    1.0287    2.2551];
waypoints2=[         0         0
   -0.5165   -1.3443
   -1.2427   -2.6482
   -2.3299   -2.9325
   -2.3566   -2.6838
   -2.0364   -2.6117
   -2.9643   -2.2900
   -6.1200   -4.2623
   -7.8462   -5.4912
   -9.7435   -6.9776
  -10.0559   -7.8263
  -10.0906   -9.4392
   -7.1065  -10.1383
   -7.4857   -9.0418
   -4.5079   -3.2495
   -2.6102   -2.9429
   -2.4789   -1.2889];
waypoints3=[         0         0
    2.4741   -1.8954
    5.0216   -3.6436
    7.0758   -3.6681
    7.7119   -2.8270
    8.0457   -2.8861
    8.6455   -4.2876
    8.3885   -8.4409
    7.9913   -6.6198
    6.3069   -5.6286
    6.3090   -8.0032
    3.2098   -8.1217
    3.3343   -6.8655
    4.1656   -5.1963
    2.9698   -3.7578
    1.7391   -1.3785
    1.5139   -2.6404];
waypoints4=[         0         0
   -3.2768    2.1892
   -6.6460    4.2976
   -7.4700    3.1829
   -8.7772    4.3214
   -8.1437    6.9970
   -7.9433    8.5667
   -5.6096    7.5246
   -6.4260    5.8560
   -4.3059    8.7033
   -2.9616    8.2505
   -3.7351    7.2497
   -4.6018    5.7769
   -5.1608    4.2399
   -3.1153    4.0119
   -1.3935    2.7478
   -1.6303    1.2664];
% waypoints1=[         0         0
%     1.3110   -2.5730
%     2.8536   -3.6045
%     4.1251   -5.1270
%     3.4833   -6.9587
%     3.1700   -8.3324
%     6.2655   -8.0768
%     8.4669   -8.6407
%     8.1339   -7.0638
%     8.5739   -4.7441
%     8.5928   -3.1657
%     7.3271   -2.9564
%     6.9044   -4.3002
%     6.2544   -5.7536
%     5.0195   -3.6430
%     2.5008   -2.1246
%     1.8014   -1.0351];
% waypoints2=[         0         0
%    -1.7885   -0.6485
%    -1.8550   -1.7943
%    -3.2054   -1.6096
%    -5.3909   -3.4101
%    -7.5726   -5.1958
%    -9.7523   -6.9760
%   -10.0971   -8.4723
%   -10.1052  -10.0603
%    -7.2138  -10.2570
%    -7.1892   -9.1092
%    -4.9983   -6.2174
%    -2.7966   -3.3341
%    -2.6383   -2.5304
%    -1.5626   -3.1474
%    -0.7569   -2.2087
%     0.0064   -1.1506];
% waypoints3=[         0         0
%     1.2593    0.5237
%     2.4187    0.9538
%     2.1293    1.8927
%     2.7465    2.9202
%     4.8311    4.4549
%     6.8809    6.0761
%     8.9158    7.7503
%    10.4140    7.6449
%    10.3993    9.9322
%     8.8483    9.8147
%     7.4437   10.4831
%     7.3387    9.1177
%     5.3626    7.1116
%     3.3177    5.0719
%     1.1320    2.9581
%     0.2864    1.5964];
% waypoints4=[         0         0
%    -1.9273    1.1041
%    -3.3468    2.4064
%    -5.0285    3.8940
%    -6.6597    4.4287
%    -7.6776    3.3268
%    -8.9600    4.2684
%    -6.6565    5.8905
%    -8.2812    7.2050
%    -7.8562    8.6780
%    -5.7378    7.3682
%    -4.3742    8.8037
%    -3.0227    8.1657
%    -4.0611    7.1279
%    -4.7255    5.5846
%    -3.0837    4.2248
%    -1.2921    2.6098];
% [x1,y1] = hilbert(2);
% waypoints1=[0 ,0;10*x1'+5,10*y1'+5];
   G.nwp1 = size(waypoints1,1);
% [x2,y2] = hilbert(2);
% waypoints2=[0 ,0;10*x2'-5,10*y2'-5];
  G.nwp2 = size(waypoints2,1);
% [x3,y3] = hilbert(2);
% waypoints3=[0 ,0;10*x3'+5,10*y3'-5];
  G.nwp3 = size(waypoints3,1);
% [x4,y4] = hilbert(2);
% waypoints4=[0 ,0;10*x4'-5,10*y4'+5];
 WAYPOINTS=[waypoints1;waypoints2;waypoints3;waypoints4];

set(G.fig,'Name','IC path controller');
%make out interesting function
szInteresting = 200;
phi = zeros(szInteresting,szInteresting);
%these are interesting!!!
[mX,mY] = meshgrid(linspace(-11,11,szInteresting),linspace(-11,11,szInteresting)); %making the grid
cellsz = mY(2) - mY(1);
% sensors=[0,0;-10,10;-10,8;-8,10;-10,-10;-10,-8;-8,-10;10,-10;-5,0;0,-5;5,0;0,5;-8,0;0,-8;8,0;0,8;8,-10;10,-8;10,-10;-3,10;3,10;-3,-10;3,-10;10,3;10,-3;-10,3;-10,-3;10,10;8,10;10,8;-2,-2;-2,2;2,-2;2,2;-4,-4;-4,4;4,-4;4,4;-6,6;6,-6;-6,-6;6,6];%Intresting points
% sensors=[0,0;-2,2;2,2;-2,-2;2,-2;3,0;0,3;-3,0;0,-3;0,5;0,-5;-5,0;0,-5;5,0;7,0;0,7;-7,0;0,-7;5,-5;-5,5;5,5;-5,-5;-10,-10;10,10;-8,-10;-10,-8;8,10;10,8;-10,10;-10,8;-8,10;10,-10;10,-8;8,-10];%Intresting points
rng(10); %seed the random number generator,  rng(10); is nice for sensors = [0,0; 20*(rand(15,2)-.5)];
%sensors = [0,0; 20*(rand(15,2)-.5)];
sensors=[0,0;-2,2;2,2;-2,-2;2,-2;-6,6;-8,8;-4,8;-8,4;-4,4;6,-6;8,-8;4,-8;8,-4;4,-4;-10,-10;10,10;-8,-10;-10,-8;8,10;10,8];%Intresting points
for m = 1:size(sensors,1)
    h = 1.25;
    phi=phi+TransmissionCostCalc(h,sensors(m,1), sensors(m,2),6, mX, mY);
end

G = plotWaypoints(WAYPOINTS,mX,mY,phi,0,G);
nwaypts = size(WAYPOINTS,1);

indExciting = find(phi>0); %index of every 'exciting point'
% constants for cost funtions:
Wn=1;
Ws = 1;
Ki=1; %potentially -time varying positive definite matrix

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
    sumSqDist = sum((repmat(pos,nwaypts,1) - WAYPOINTS).^2,2);
    dists(m,:) = sumSqDist;
    [minDist,minWP] = min(sumSqDist);
    minDists(m) = minDist;
    minWPs(m) = minWP;
end

tic
maxIters = 100;
costs = zeros(maxIters+1,1);
costs(1) = calcCost( minDists,phi,indx,Ws,Wn,WAYPOINTS);
moveableWP = [2:G.nwp1,G.nwp1+2:G.nwp1+G.nwp2,G.nwp1+G.nwp2+2:G.nwp1+G.nwp2+G.nwp3,G.nwp1+G.nwp2+G.nwp3+2:nwaypts];
for iter = 1:maxIters
    for wp=moveableWP %iterate through all waypoints
        smallerInd = dists(:,wp)<=minDists';
        
        M_val = sum(phi(smallerInd)); %calculate the mass
        L_val = sum(poses(smallerInd,:).*repmat(phi(smallerInd),1,2)); %calculate the mass
        
        %calculate errors between centroid and desired position
        if M_val == 0
            e_val=[0,0];
        else
            C_val = L_val./[M_val,M_val];
            e_val= C_val - [WAYPOINTS(wp,1),WAYPOINTS(wp,2)];
        end
        
        if wp ==nwaypts % be careful to wrap the last waypoint back to the first
            alpha_val=Wn*(WAYPOINTS(wp-1,:)+WAYPOINTS(1,:)-2*WAYPOINTS(wp,:));
        else
            alpha_val=Wn*(WAYPOINTS(wp-1,:)+WAYPOINTS(wp+1,:)-2*WAYPOINTS(wp,:));
        end
        beta_val=M_val+(2*Wn);
        
        % moving towards the centeroid of the voronoi cells
        u=Ki.*((M_val.*e_val)+alpha_val)/beta_val; %Control input based on gradient descent
        if ~isequal(u,[0,0])
            %apply control input
            deltat=0.1; %time period
            WAYPOINTS(wp,:)=WAYPOINTS(wp,:)+u*deltat; %updating waypoints
            %recalculate if this waypoint moved
            dists(:,wp) = sum( (poses - repmat(WAYPOINTS(wp,:),numel(indExciting),1)).^2,2);
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
        
    end
W=iter;
    G = plotWaypoints(WAYPOINTS,mX,mY,phi,W,G); % calling function
    %TODO:  make a flag to decide whether to do ANY plotting
    
    costs(iter+1) = calcCost( minDists,phi,indx,Ws,Wn,WAYPOINTS);
end
toc
disp(WAYPOINTS);
if MAKE_MOVIE
    close(G.writerObj);
end

figure(2)
clf;
plot(costs)
disp(costs);
xlabel('iteration')
ylabel('Cost function')
end
function cost = calcCost( minDists,phi,indx,Ws,Wn,WAYPOINTS)

cost = ( Ws/2*sum(minDists*phi(indx))...
    + Wn/2*sum( (  ...
     (WAYPOINTS(:,1) - [WAYPOINTS(2:end,1);WAYPOINTS(1,1)]).^2 +... %x-distances
    (WAYPOINTS(:,2) - [WAYPOINTS(2:end,2);WAYPOINTS(1,2)]).^2 ).^1 ...  % ydistances
    ));

end


function G = plotWaypoints(WAYPOINTS,mX,mY,phi,iter,G)
%         pcolor(mX,mY,phi) 
if ~G.init
    G.hPhiPlot = imagesc(mX(1,:),mY(:,1),phi);
    set(gca,'YDir','normal');
    %colormap lines
    colormap copper
    %         colormap([0,0,0;1,0,0])
    hold on
%    G.hPath = line([WAYPOINTS(:,1);WAYPOINTS(1,1)],[WAYPOINTS(:,2);WAYPOINTS(1,2)],'color','m');
%     set(G.hPath, 'linewidth',2);
%     G.hWaypoints = plot(WAYPOINTS(:,1),WAYPOINTS(:,2),'go');
    G.hPath1 = line(WAYPOINTS(1:G.nwp1+1,1),WAYPOINTS(1:G.nwp1+1,2),'color','w');
    G.hPath2 = line([WAYPOINTS(G.nwp1+1:G.nwp1+G.nwp2,1);WAYPOINTS(1,1)],[WAYPOINTS(G.nwp1+1:G.nwp1+G.nwp2,2);WAYPOINTS(1,2)],'color','r');
    G.hPath3 = line([WAYPOINTS(G.nwp1+G.nwp2+1:G.nwp1+G.nwp2+G.nwp3,1);WAYPOINTS(1,1)],[WAYPOINTS(G.nwp1+G.nwp2+1:G.nwp1+G.nwp2+G.nwp3,2);WAYPOINTS(1,2)],'color','c');
    G.hPath4 = line([WAYPOINTS(G.nwp1+G.nwp2+G.nwp3+1:end,1);WAYPOINTS(1,1)],[WAYPOINTS(G.nwp1+G.nwp2+G.nwp3+1:end,2);WAYPOINTS(1,2)],'color','m');
    set(G.hPath1, 'linewidth',2);
    set(G.hPath2, 'linewidth',2);
     set(G.hPath3, 'linewidth',2);
      set(G.hPath4, 'linewidth',2);
    G.hWaypoints1 = plot(WAYPOINTS(1:G.nwp1,1),WAYPOINTS(1:G.nwp1,2),'go');
    G.hWaypoints2 = plot(WAYPOINTS(G.nwp1+1:G.nwp1+G.nwp2,1),WAYPOINTS(G.nwp1+1:G.nwp1+G.nwp2,2),'go');
     G.hWaypoints3 = plot(WAYPOINTS(G.nwp1+G.nwp2+1:G.nwp1+G.nwp2+G.nwp3,1),WAYPOINTS(G.nwp1+G.nwp2+1:G.nwp1+G.nwp2+G.nwp3,2),'go');
      G.hWaypoints4 = plot(WAYPOINTS(G.nwp1+G.nwp2+G.nwp3+1:end,1),WAYPOINTS(G.nwp1+G.nwp2+G.nwp3+1:end,2),'go');
    axis equal
    axis tight
    a = axis;
    [vx,vy] = voronoi(WAYPOINTS(:,1),WAYPOINTS(:,2));
    G.hVoronoi = plot(vx,vy,'b-');
    uistack(G.hVoronoi,'bottom');
    axis(a);  % voronoi tends to make the axis zoom out.
    uistack(G.hPhiPlot, 'bottom')
    
%      title '400';
   G.htitle = title(num2str(iter));
    xlabel 'X-axis(m)'
    ylabel 'Y-axis(m)'
    G.init =true;
else
    set(G.hPhiPlot,'Cdata',phi);
    set(G.hWaypoints1,'Xdata',WAYPOINTS(1:G.nwp1,1),'Ydata',WAYPOINTS(1:G.nwp1,2));
    set(G.hWaypoints2,'Xdata',WAYPOINTS(G.nwp1+1:G.nwp1+G.nwp2,1),'Ydata',WAYPOINTS(G.nwp1+1:G.nwp1+G.nwp2,2));
    set(G.hWaypoints3,'Xdata',WAYPOINTS(G.nwp1+G.nwp2+1:G.nwp1+G.nwp2+G.nwp3,1),'Ydata',WAYPOINTS(G.nwp1+G.nwp2+1:G.nwp1+G.nwp2+G.nwp3,2));
    set(G.hWaypoints4,'Xdata',WAYPOINTS(G.nwp1+G.nwp2+G.nwp3+1:end,1),'Ydata',WAYPOINTS(G.nwp1+G.nwp2+G.nwp3+1:end,2));
    set(G.hPath1,'Xdata',[WAYPOINTS(1:G.nwp1,1);WAYPOINTS(1,1)],'Ydata',[WAYPOINTS(1:G.nwp1,2);WAYPOINTS(1,2)]);
    set(G.hPath2,'Xdata',[WAYPOINTS(G.nwp1+1:G.nwp1+G.nwp2,1);WAYPOINTS(1,1)],'Ydata',[WAYPOINTS(G.nwp1+1:G.nwp1+G.nwp2,2);WAYPOINTS(1,2)]);
    set(G.hPath3,'Xdata',[WAYPOINTS(G.nwp1+G.nwp2+1:G.nwp1+G.nwp2+G.nwp3,1);WAYPOINTS(1,1)],'Ydata',[WAYPOINTS(G.nwp1+G.nwp2+1:G.nwp1+G.nwp2+G.nwp3,2);WAYPOINTS(1,2)]);
    set(G.hPath4,'Xdata',[WAYPOINTS(G.nwp1+G.nwp2+G.nwp3+1:end,1);WAYPOINTS(1,1)],'Ydata',[WAYPOINTS(G.nwp1+G.nwp2+G.nwp3+1:end,2);WAYPOINTS(1,2)]);
    set(G.htitle,'string', num2str(iter));
    %computing the waypoint's Voronoi Partition
    % [vx,vy] = voronoi(waypoints(:,1),waypoints(:,2));
    % set(G.hVoronoi,'Xdata',vx,'Ydata',vy)
    
    
    a = axis;
    [vx,vy] = voronoi(WAYPOINTS(:,1),WAYPOINTS(:,2));
    delete(G.hVoronoi)
    G.hVoronoi = plot(vx,vy,'b-');
    uistack(G.hVoronoi,'bottom');
    axis(a);  % voronoi tends to make the axis zoom out.
    uistack(G.hPhiPlot,'bottom')
    
    
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
function [O] = TransmissionCostCalc(h, xs, ys, ~, xr, yr)
%  h is height of robot
% [xs,ys] center of sensor nodes,
% [xr,yr] are grid points
%Defining a function for the intresting points
% We would use it in waypointControlForPathPlaning as a function that
% generates intresting points for robots
%O = 1./(((h)^(2))+((xs-xr).^(2))+((ys-yr).^(2))).^((pow)/(2));
 O =   1* ( ((xs-xr).^2+(ys-yr).^2)<3); %Binary cost -- is it better?
end
% set(gcf,'papersize',[6,6]);
% set(gcf,'paperposition',[0,0,6,6]);
% print -dpdf '2_1.pdf'