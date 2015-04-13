function TwoRobotWaypointPathPlanning()
%Using the HILBERT CURVE FOR INITIALIZING WAYPOINTS
 [x1,y1] = hilbert(3);
  waypoints1=20*[x1',y1'];
  [x2,y2] = hilbert(3);
  waypoints2=10*[x2',y2'];
%  n1=10;
%   theta1 = linspace(0,2*pi*(1-1/n1),n1)';
%   waypoints1 = 5*[cos(theta1),sin(theta1)];
%   n2 = 10;
%   theta2 = linspace(0,2*pi*(1-1/n2),n2)';
%   waypoints2 = 10*[cos(theta2),sin(theta2)];
WAYPOINTS=[waypoints1;0 0;waypoints2;0 0];
disp(WAYPOINTS);
f = figure(1);
set(f,'Name','IC path controller');
%make out interesting function
szInteresting = 100;
phi = zeros(szInteresting,szInteresting);
%these are interesting!!!
[mX,mY] = meshgrid(linspace(-10,10,szInteresting),linspace(-10,10,szInteresting)); %making the grid
cellsz = mY(2) - mY(1);
sensors1=[0,0;-2,2;2,2;-2,-2;2,-2;-6,6;-8,8;-4,8;-8,4;-4,4;6,-6;8,-8;4,-8;8,-4;4,-4;-10,-10;10,10;-8,-10;-10,-8;8,10;10,8];%Intresting points
for m = 1:size(sensors1,1)
    h = .5;
    phi=phi+TransmissionCost(h,sensors1(m,1), sensors1(m,2),6, mX, mY);
end
plotWaypoints(WAYPOINTS,mX,mY,phi);


for iter = 1:2000 % here for these iterations we calculate the previous and next neighbour for each waypoint
    H1=[WAYPOINTS(:,1),WAYPOINTS(:,2)];
    pim1=[WAYPOINTS(end,:);WAYPOINTS(1:end-1,:)];
    pip1=[WAYPOINTS(2:end,:);WAYPOINTS(1,:)];
    
    
    nwaypts1 = size(WAYPOINTS,1);
    M_vals1 = zeros(size(WAYPOINTS,1),1); %mass
    L_vals1 = zeros(size(WAYPOINTS,1),2); %first mass moment
    C_vals1 = zeros(size(WAYPOINTS,1),2); %centroids
    
    indExciting = find(phi>0); %index of every 'exciting point'
    for m = 1:numel(indExciting)  % iterate through every grid point that is 'exciting'
        indx = indExciting(m);
        pos = [mX(indx)+cellsz/2,mY(indx)+cellsz/2];  %center of grid cell is [mX,mY] + 1/2[cellsize,cellsize], pos is the center of the interesting gridcell
        
        %squared distance between this grid cell and every waypoint
        sumSqDist1 = sum((repmat(pos,nwaypts1,1) - WAYPOINTS).^2,2);
        [~,minIndx1] = min(sumSqDist1);
        
        M_vals1(minIndx1) = M_vals1(minIndx1)+phi(indx); %calculate the mass
        L_vals1(minIndx1,:) = L_vals1(minIndx1,:)+pos*phi(indx); %calcuate the mass
    end
    %calculate errors between ccentroid and desired position
    e_vals1 = zeros(size(WAYPOINTS,1),2);
    C_vals1 = L_vals1./[M_vals1,M_vals1];
    
    for i= 1:nwaypts1        %errors
        
        if isnan(C_vals1(i,1))
            e_vals1(i,:)=[0,0];
        else
            e_vals1(i,:)= C_vals1(i,:) - H1(i,:);
        end
        
    end
    
    Wn=1;
    alpha_vals1=zeros(size(WAYPOINTS,1),2);
    beta_vals1=zeros(size(WAYPOINTS,1),1);
    uir1 =zeros(size(WAYPOINTS,1),2);
    
    for i = 1:nwaypts1
        alpha_vals1(i,:)=Wn*(pim1(i,:)+pip1(i,:)-2*H1(i,:));
        beta_vals1(i,1)=M_vals1(i,:)+(2*Wn);
    end
    % moving towards the centeroid of the voronoi cells
    Ki=1; %potentially -time varying positive definite matrix
    for i = 1:nwaypts1
        if(WAYPOINTS(:,1)==0 )
            uir1(i,:)=0;
        end
        uir1(i,:)=Ki.*((M_vals1(i,:).*e_vals1(i,:))+alpha_vals1(i,:))/beta_vals1(i,1); %Control input based on gradient descent
    end
    %apply control input
    deltat=0.1; %time period
    WAYPOINTS=WAYPOINTS+uir1*deltat; %updating waypoints
    plotWaypoints(WAYPOINTS,mX,mY,phi) %calling function
    title(num2str(iter))
    pause(0.000001)
    
end

    function plotWaypoints(WAYPOINTS,mX,mY,phi)
        pcolor(mX,mY,phi)
%                 imagesc(mX(1,:),mY(:,1),phi);
%                  set(gca,'YDir','normal');
%                  colormap copper;
        colormap([0,0,0;0,0,1])
        hold on
        hWaypoints1 = plot(WAYPOINTS(1:size(waypoints1)+1,1),WAYPOINTS(1:size(waypoints1)+1,2),'go');
        hWaypoints2 = plot(WAYPOINTS(size(waypoints1)+2:end,1),WAYPOINTS(size(waypoints1)+2:end,2),'ro');
        axis equal
        axis tight
        hPath1 = line([WAYPOINTS(:,1);WAYPOINTS(1,1)],[WAYPOINTS(:,2);WAYPOINTS(1,2)],'color','m');
        %computing the waypoint's Voronoi Partition
        voronoi(WAYPOINTS(:,1),WAYPOINTS(:,2));
    end

end