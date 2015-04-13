
function TwoRobotWaypointPathPlanningCostfunc()
%Using the HILBERT CURVE FOR INITIALIZING WAYPOINTS
%   [x1,y1] = hilbert(3);
%    waypoints1=20*[x1',y1'];
%    [x2,y2] = hilbert(3);
%    waypoints2=10*[x2',y2'];
n1=20;
theta1 = linspace(5/4*pi,13/4*pi*(1-1/n1),n1)';
waypoints1 = [0,0;cos(theta1)+5,sin(theta1)+5];
n2 = 20;
theta2 = linspace(1/4*pi,9/4*pi*(1-1/n2),n2)';
waypoints2 = [0,0;cos(theta2)-5,sin(theta2)-5];
WAYPOINTS=[waypoints1;waypoints2];

%make out interesting function
szInteresting = 100;
phi = zeros(szInteresting,szInteresting);
%these are interesting!
[mX,mY] = meshgrid(linspace(-11,11,szInteresting),linspace(-11,11,szInteresting)); %making the grid
cellsz = mY(2) - mY(1);
sensors1=[0,0;-2,2;2,2;-2,-2;2,-2;-6,6;-8,8;-4,8;-8,4;-4,4;6,-6;8,-8;4,-8;8,-4;4,-4;-10,-10;10,10;-8,-10;-10,-8;8,10;10,8];%Interesting points  
for m = 1:size(sensors1,1)
    h = .5;
    phi=phi+TransmissionCost(h,sensors1(m,1), sensors1(m,2),6, mX, mY);
end
costfunc=[];
for iter = 1:1000 % here for these iterations we calculate the previous and next neighbour for each waypoint

    if(mod(iter,10)==0)
       display(['Iteration ',num2str(iter)])
    end
    nwaypts = size(WAYPOINTS,1);
    M_vals = zeros(nwaypts,1); %mass
    L_vals = zeros(nwaypts,2); %first mass moment
    
    indExciting = find(phi>0); %index of every 'exciting point'
    minCosts = zeros(size(indExciting));
    for m = 1:numel(indExciting)  % iterate through every grid point that is 'exciting'
        indx = indExciting(m);
        pos = [mX(indx)+cellsz/2,mY(indx)+cellsz/2];  %center of grid cell is [mX,mY] + 1/2[cellsize,cellsize], pos is the center of the interesting gridcell
        
        %squared distance between this grid cell and every waypoint
        sumSqDist1 = sum((repmat(pos,nwaypts,1) - WAYPOINTS).^2,2);
        [minDist,minIndx1] = min(sumSqDist1);
   
        minCosts(m) =  minDist^2*phi(indx);
        M_vals(minIndx1) = M_vals(minIndx1)+phi(indx); %calculate the mass
        L_vals(minIndx1,:) = L_vals(minIndx1,:)+pos*phi(indx); %calcuate the mass
    end
    %calculate errors between ccentroid and desired position
    e_vals = zeros(size(WAYPOINTS,1),2);
    C_vals = L_vals./[M_vals,M_vals];
    
    %for i= 1:nwaypts        %errors
     for i= mod(iter,nwaypts)+1
        if isnan(C_vals(i,1))
            e_vals(i,:)=[0,0];
        else
            e_vals(i,:)= C_vals(i,:) - WAYPOINTS(i,:);
        end
    end
    
    Wn=1;

    %Vectorized this for loop:
    indm1 = [nwaypts,1:nwaypts-1];
    indp1 = [2:nwaypts,1];

    alpha_vals =Wn*(WAYPOINTS(indm1,:)+WAYPOINTS(indp1,:)-2*WAYPOINTS);
    beta_vals=M_vals+(2*Wn);
 
    % moving towards the centeroid of the voronoi cells
    Ki=1; %potentially time-varying positive definite matrix

    uir1=Ki.*([M_vals,M_vals].*e_vals+alpha_vals)./[beta_vals,beta_vals]; %Control input based on gradient descent

    % force the waypoints at the SINK to be stationary:
    uir1(1,:) = [0,0];
    uir1(nwaypts/2+1,:) = [0,0];
    
    %apply control input
    deltat=0.1; %time period
    WAYPOINTS=WAYPOINTS+uir1*deltat; %updating waypoints
%%%CALCULATION OF COST FUNCTION
Ws=1;
Wn=1;
P=(Ws/2*(sum(minCosts))+sum(Wn/2*((WAYPOINTS(:,1)-WAYPOINTS(indp1,1)).^2)+((WAYPOINTS(:,2)-WAYPOINTS(indp1,2)).^2))); 
costfunc = [costfunc;P];
end
% disp(size(costfunc));
% disp(costfunc);
 d=1:1000;
 plot(d,costfunc,'b');
title 'Two Robot Waypoint Path Planning';
xlabel 'ITERATION NUMBER';
ylabel 'COST FUNCTION';

end