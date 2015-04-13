function costs = waypointContolForPathPlanningFASTER()
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

G = plotWaypoints(waypoints,mX,mY,phi,0,G);
nwaypts = size(waypoints,1);

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
        
    end
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
plot(costs)
xlabel('iteration')
ylabel('Cost function')
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
    %uistack(G.hVoronoi,'bottom');
    axis(a);  % voronoi tends to make the axis zoom out.
    uistack(G.hPhiPlot, 'bottom')
    
    
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
    % [vx,vy] = voronoi(waypoints(:,1),waypoints(:,2));
    % set(G.hVoronoi,'Xdata',vx,'Ydata',vy)
    
    
    a = axis;
    [vx,vy] = voronoi(waypoints(:,1),waypoints(:,2));
    delete(G.hVoronoi)
    G.hVoronoi = plot(vx,vy,'b-');
    %uistack(G.hVoronoi,'bottom');
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
