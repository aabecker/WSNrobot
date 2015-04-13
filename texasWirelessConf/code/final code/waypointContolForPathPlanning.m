function waypointContolForPathPlanning()
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
% waypoints=[    0         0;
%    -1.4103   -1.4036;
%    -1.5774   -3.2198;
%    -2.1290   -4.3240;
%    -2.7506   -5.3446;
%    -3.4696   -6.2794;
%    -4.3128   -7.1247;
%    -5.2817   -7.8818;
%    -6.3383   -8.5502;
%    -7.5732   -7.6390;
%    -8.7822   -8.7781;
%    -8.8214   -6.7290;
%    -3.3352   -1.4338;
%    -4.4518   -1.2664;
%    -5.3918   -0.9740;
%    -6.0971   -0.4845;
%    -6.5467    0.2404;
%    -6.7649    1.1923;
%    -6.8137    2.3257;
%    -6.7465    3.6164;
%    -8.5657    3.8649;
%    -8.6142    6.2759;
%    -8.5604    8.5799;
%    -6.6432    8.4037;
%    -6.1519    6.2698;
%    -4.2445    6.8034;
%    -3.7549    8.5836;
%    -2.6471    7.5007;
%    -2.2178    5.8078;
%    -2.0236    4.1187;
%    -3.7337    3.7823;
%    -3.1627    1.7766;
%    -1.3403    1.4059;
%     1.3744    1.3989;
%     3.2468    1.6140;
%     3.1048    3.1609;
%     2.1924    4.0260;
%     2.5437    4.9674;
%     3.1042    5.8638;
%     3.8381    6.6699;
%     4.7201    7.3765;
%     5.7213    8.0021;
%     6.8122    8.5750;
%     8.8085    8.8028;
%     8.7023    6.5868;
%     8.5058    4.9035;
%     8.3634    3.2317;
%     8.3096    1.5504;
%     8.3493   -0.1589;
%     8.4608   -1.9057;
%     8.6430   -3.7202;
%     8.3702   -6.6305;
%     8.4995   -8.4442;
%     6.2708   -6.2575;
%     5.5908   -8.0842;
%     3.8042   -8.6483;
%     2.9088   -7.9409;
%     2.9334   -6.9950;
%     3.4595   -5.6960;
%     6.6250   -3.4301;
%     5.0977   -2.4234;
%     3.7411   -3.7990;
%     1.7553   -3.2320;
%     3.5202   -1.4572;
%     1.3822   -1.3558];
f = figure(1);
global MAKE_MOVIE G
MAKE_MOVIE = false;
if MAKE_MOVIE
    MOVIE_NAME = 'WAY POINT CONTROL FOR PATH PLANNING';
    G.fig = f;
    clf
    set(G.fig,'Units','normalized','outerposition',[0 0 1 1],'NumberTitle','off','MenuBar','none','color','w');
    G.writerObj = VideoWriter(MOVIE_NAME,'MPEG-4');%http://www.mathworks.com/help/matlab/ref/videowriterclass.html
    set(G.writerObj,'Quality',100);
    open(G.writerObj);
end



set(f,'Name','IC path controller');
%make out interesting function
szInteresting = 100;
phi = zeros(szInteresting,szInteresting);
%these are interesting!!!
[mX,mY] = meshgrid(linspace(-11,11,szInteresting),linspace(-11,11,szInteresting)); %making the grid
cellsz = mY(2) - mY(1);
sensors=[0,0;-2,2;2,2;-2,-2;2,-2;-6,6;-8,8;-4,8;-8,4;-4,4;6,-6;8,-8;4,-8;8,-4;4,-4;-10,-10;10,10;-8,-10;-10,-8;8,10;10,8];%Intresting points
for m = 1:size(sensors,1)
    h = .5;
    phi=phi+TransmissionCost(h,sensors(m,1), sensors(m,2),6, mX, mY);
end

plotWaypoints(waypoints,mX,mY,phi,0);
nwaypts = size(waypoints,1);

indExciting = find(phi>0); %index of every 'exciting point'
tic
for iter = 1:5
    
    for wp=2:nwaypts %iterate through all waypoints
        H1=[waypoints(:,1),waypoints(:,2)];
        
        M_val = 0;
        L_val = 0;
        
        for m = 1:numel(indExciting)  % iterate through every grid point that is 'exciting'
            indx = indExciting(m);
            
            pos = [mX(indx)+cellsz/2,mY(indx)+cellsz/2];  %center of grid cell is [mX,mY] + 1/2[cellsize,cellsize], pos is the center of the interesting gridcell
            
            
            %squared distance between this grid cell and every waypoint
            sumSqDist = sum((repmat(pos,nwaypts,1) - waypoints).^2,2);
            [~,minIndx] = min(sumSqDist);
            if minIndx == wp
                M_val = M_val+phi(indx); %calculate the mass
                L_val = L_val+pos*phi(indx); %calculate the mass
            end
        end
        %calculate errors between ccentroid and desired position
        
        C_val = L_val./[M_val,M_val];
        
        %      for i= 1:nwaypts        %errors
        
        if isnan(C_val)
            e_val=[0,0];
        else
            e_val= C_val - [waypoints(wp,1),waypoints(wp,2)];
        end
        
        
        Wn=1;
        
        if wp ==nwaypts
            alpha_val=Wn*(waypoints(wp-1,:)+waypoints(1,:)-2*waypoints(wp,:));
        else
            alpha_val=Wn*(waypoints(wp-1,:)+waypoints(wp+1,:)-2*waypoints(wp,:));
        end
        beta_val=M_val+(2*Wn);
        
        % moving towards the centeroid of the voronoi cells
        Ki=1; %potentially -time varying positive definite matrix
        
        
        %         uir((nwaypts+1)/2,:) = [0,0];
        u=Ki.*((M_val.*e_val)+alpha_val)/beta_val; %Control input based on gradient descent
        %apply control input
        deltat=0.1; %time period
        waypoints(wp,:)=waypoints(wp,:)+u*deltat; %updating waypoints
        
        %         if ( iter<3 && mod(wp,2)) || mod(wp,10)
        %             plotWaypoints(waypoints,mX,mY,phi,iter) % calling function
        %         end
        
    end
    
    
    plotWaypoints(waypoints,mX,mY,phi,iter) % calling function
    
    %TODO:  make a flag to decide whether to do ANY plotting
    
end
toc
disp(waypoints);
if MAKE_MOVIE
    close(G.writerObj);
end
end

function plotWaypoints(waypoints,mX,mY,phi,iter)
%         pcolor(mX,mY,phi)
imagesc(mX(1,:),mY(:,1),phi)
set(gca,'YDir','normal');
%colormap lines
colormap copper
%         colormap([0,0,0;1,0,0])
hold on
hWaypoints = plot(waypoints(:,1),waypoints(:,2),'go');
axis equal
axis tight
hPath = line([waypoints(:,1);waypoints(1,1)],[waypoints(:,2);waypoints(1,2)],'color','m');
%computing the waypoint's Voronoi Partition
voronoi(waypoints(:,1),waypoints(:,2));

title(num2str(iter))
xlabel 'X-axis(m)'
ylabel 'Y-axis(m)'
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
