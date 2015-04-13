function DecentralizedPathPlanningForCoverageUsingGradientDescent1()
% Given a 2D map with so-called 'interesting regions' defined by a function phi(x,y), this algorithm
% searches for a locally-optimum closed loop that covers the interesting region(s)
% with a short path.  The algorithm works by iteratively moving a set of
% waypoints.  The waypoints have two control primitives: (1.)  move ith
% waypoint  toward the centroid of its Voronoi region, weighted by the  phi(x,y)
%  function, and (2.) move waypoint i to be halfway between waypoints i-1
%  and i+1.
%
% This implementation is based on 'Algorithm 1' from the paper
%  @article{SolteroEtAlIJRR14VoronoiPathPlanning,
%   author = {D. E. Soltero and M. Schwager and D. Rus},
%   title = {Decentralized path planning for coverage tasks using gradient descent adaptive control},
%   journal = {International Journal of Robotics Research},
%   month = {March},
%   year = {2014},
%   volume = {33},
%   number = {3},
%   pages = {401--425}}
%
% Authors: Srikanth K. V. Sudarshan and Aaron T. Becker
% Date 2/11/2015
%
% TODO: vectorize computation for speed
%   instead of replotting in plotWaypoints, just update plot data
%   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% INITIALIZE WAYPOINTS TO BE IN A CIRCLE.
numWaypoints = 10;
theta = linspace(0,2*pi*(1-1/numWaypoints),numWaypoints)';
workspaceRadius = 10;
waypoints = workspaceRadius*[cos(theta),sin(theta)];
% INTERESTING CLOSED PATH CONTROLLER
f = figure(1);
set(f,'Name','IC path controller');
%set the 'interesting function' to be all cells in lower left corner
szInteresting = 100;

[mX,mY] = meshgrid(linspace(-workspaceRadius,workspaceRadius,szInteresting),linspace(-workspaceRadius,workspaceRadius,szInteresting)); %making the grid
cellsz = mY(2) - mY(1);

%setting  points to be interesting
%phi = zeros(szInteresting,szInteresting);
%phi(1:25,1:25) = 1; %sets a square patch to be 1
% set a mixutre model of Gaussians as the interesting region:
mXl = reshape(mX,numel(mX),1);
mYl = reshape(mY,numel(mY),1);
phi = normPdfMAT([mXl,mYl],workspaceRadius*[-.8,.8],workspaceRadius*[.5,0;0,.5])...
    +0.7*normPdfMAT([mXl,mYl],workspaceRadius*[-.3,.8],workspaceRadius*[.5,0;0,.5])...
    +0.5*normPdfMAT([mXl,mYl],workspaceRadius*[-.3,.3],workspaceRadius*[.5,0;0,.5]);

phi = reshape(phi,szInteresting,szInteresting);

for iter = 1:350 % here for these iterations we calculate the previous and next neighbouring waypoint for each waypoint
    H1=[waypoints(:,1),waypoints(:,2)];
    pim=[waypoints(end,:);waypoints(1:end-1,:)];
    pip=[waypoints(2:end,:);waypoints(1,:)];
    
    nwaypts = size(waypoints,1); %size of the waypoint matrix
    M_vals = zeros(size(waypoints,1),1); %mass
    L_vals = zeros(size(waypoints,1),2); %first mass moment
    
    % doing a for loop over every entry in phi and assigning this value to the mass of the correct waypoint.
    indExciting = find(phi>0); %index of every 'exciting point'
    for m = 1:numel(indExciting)  % iterating through every grid point that is 'exciting'
        indx = indExciting(m);
        pos = [mX(indx)+cellsz/2,mY(indx)+cellsz/2];  %center of grid cell is [mX,mY] + 1/2[cellsize,cellsize], pos is the center of the interesting gridcell
        
        %squared distance between this grid cell and every waypoint
        sumSqDist = sum((repmat(pos,nwaypts,1) - waypoints).^2,2);
        [~,minIndx] = min(sumSqDist);
        
        M_vals(minIndx) = M_vals(minIndx)+phi(indx); %calculate the mass
        L_vals(minIndx,:) = L_vals(minIndx,:)+pos*phi(indx); %calcuate the mass
    end
    
    e_vals = zeros(size(waypoints,1),2); %calculating errors between ccentroid and desired position
    C_vals = L_vals./[M_vals,M_vals];
    for i= 1:nwaypts %errors values
        
        if isnan(C_vals(i,1)) %if (C_vals(i)) is not a number, set error to zero
            e_vals(i,:)=[0,0];
        else
            e_vals(i,:)= C_vals(i,:) - H1(i,:);
        end
    end
    
    Wn=1; %Is a weight the accounts for the distance between neighbouring waypoints
    alpha_vals=zeros(size(waypoints,1),2);
    beta_vals=zeros(size(waypoints,1),1);
    uir =zeros(size(waypoints,1),2);
    for i = 1:nwaypts
        alpha_vals(i,:)=Wn*(pim(i,:)+pip(i,:)-2*H1(i,:));
        beta_vals(i,1)=M_vals(i,:)+(2*Wn);
    end
    % moving towards the centeroid of the voronoi cells
    Ki=1; %potentially-time varying positive definite matrix
    for i = 1:nwaypts
        uir(i,:)=Ki.*((M_vals(i,:).*e_vals(i,:))+alpha_vals(i,:))/beta_vals(i,1); %Control input based on gradient descent
    end
    %apply control input
    deltat=1; %time period
    waypoints=waypoints+uir*deltat; %updating waypoints
    %dataWaypoints(waypoints,mX,mY,phi) % calling function
    %title(['Step ',num2str(iter)])
    %pause(0.1)
    %drawnow()
end

        %plotting the interesting region, the Voronoi Cells, the path, and the waypoints
         imagesc(mX(1,:),mY(:,1),phi)
          set(gca,'YDir','normal');
          colormap copper
          hold on
          plot(waypoints(:,1),waypoints(:,2),'go');
          axis equal
          axis tight
          line([waypoints(:,1);waypoints(1,1)],[waypoints(:,2);waypoints(1,2)],'color','m');
        %compute the waypoint's Voronoi Partition
        voronoi(waypoints(:,1),waypoints(:,2));
        xlabel 'X-axis (m)';
        ylabel 'Y-axis (m)';
    
    function px = normPdfMAT(X,Mu, Sigma)
        % returns the probability of drawing each value of X from a normal
        % distribution with mean Mu and variance Sigma.  X is an m*n
        % matrix, with m samples of a n-element random vector.  Mu is 1*n,
        % Sigma is n*n.
        Mu = repmat(Mu, size(X,1),1);
        px = det(2*pi*Sigma)^(-1/2) * exp(-1/2*sum((X-Mu)/(Sigma).*(X-Mu),2));
    end
end


