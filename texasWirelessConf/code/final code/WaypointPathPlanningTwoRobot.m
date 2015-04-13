function WaypointPathPlanningTwoRobot()

global waypoints1
   waypoints1=[0         0;
   -8.7362   -8.7197;
   -8.7125   -6.6494;
   -7.4892   -7.5502;
   -6.2573   -8.4263;
   -5.2516   -7.7894;
   -4.3358   -7.0339;
   -3.5547   -6.1536;
   -2.9166   -5.1514;
   -2.4011   -4.0425;
   -1.9791   -2.8508;
   -1.6799   -1.5815;
   -3.1031   -1.5721;
   -4.2827   -1.3978;
   -5.2761   -1.0763;
   -6.0261   -0.5533;
   -6.5063    0.1988;
   -6.7382    1.1659;
   -6.7827    2.3000;
   -6.6885    3.5712;
   -8.6277    3.8379;
   -8.6622    6.2548;
   -8.6222    8.6297;
   -6.5642    8.4385;
   -6.1793    6.2879;
   -4.2601    6.8005;
   -3.7415    8.6317;
   -2.7092    7.7168;
   -2.8210    6.3844;
   -3.0707    5.0412;
   -3.6834    3.7461;
   -2.7740    2.2228;
   -1.0055    1.7398;
    0.9185    1.7103;
    2.7998    1.8881;
    2.8236    3.0938;
    2.9245    4.1934;
    3.1453    5.1923;
    3.5329    6.0704;
    4.1109    6.8246;
    4.8734    7.4735;
    5.7896    8.0486;
    6.8175    8.5792;
    8.8085    8.8028;
    8.7027    6.5867;
    8.5068    4.9035;
    8.3650    3.2318;
    8.3111    1.5505;
    8.3504   -0.1587;
    8.4614   -1.9055;
    8.6431   -3.7203;
    6.6180   -3.4642;
    4.9910   -2.7343;
    3.3879   -2.0763;
    1.8507   -1.6739;
    2.3742   -2.8432;
    3.6653   -3.8302;
    3.4017   -5.6323;
    2.9274   -6.9897;
    2.9061   -7.9410;
    3.8042   -8.6483;
    5.5912   -8.0846;
    6.2711   -6.2578;
    8.3703   -6.6305;
    8.4995   -8.4442;];
  
waypoints2=[    0         0;
   -2.3169   -2.1815;
   -2.5049   -2.7272;
   -2.4567   -3.0618;
   -2.3187   -3.2686;
   -2.1345   -3.3365;
   -1.9524   -3.2723;
   -1.8132   -3.0722;
   -1.7389   -2.7397;
   -1.8498   -1.8987;
   -1.1919   -1.5769;
   -1.3637   -1.1457;
   -1.9094   -0.9433;
   -2.5533   -0.9518;
   -2.9504   -0.7728;
   -3.2578   -0.4879;
   -3.4871   -0.0873;
   -3.6547    0.4282;
   -3.7892    1.0457;
   -3.9250    1.7424;
   -4.0770    2.4965;
   -4.2536    3.3099;
   -4.2931    4.3136;
   -3.2705    4.2495;
   -2.6698    3.7150;
   -2.0458    3.2950;
   -1.6057    3.2116;
   -1.2720    3.0350;
   -1.0567    2.6828;
   -1.0859    2.1665;
   -1.8465    1.8797;
   -1.3054    1.1977;
   -0.5279    0.6924;
    0.5712    0.6480;
    1.3066    1.1948;
    1.8629    1.8816;
    1.2564    2.2579;
    1.2293    2.6683;
    1.3744    2.9770;
    1.6485    3.1623;
    2.0177    3.2380;
    2.5121    3.2217;
    2.9183    3.0975;
    3.2474    2.8910;
    3.4645    2.5603;
    3.5760    2.0980;
    3.6189    1.5347;
    3.6173    0.9051;
    3.5742    0.2558;
    3.4843   -0.3708;
    3.3371   -0.9391;
    3.1271   -1.4018;
    2.6529   -1.0613;
    1.8811   -0.8877;
    1.2690   -0.9756;
    1.3499   -1.4109;
    1.8513   -1.8914;
    1.7699   -2.7383;
    1.9225   -3.1500;
    2.1686   -3.4515;
    2.5091   -3.6350;
    2.9417   -3.6716;
    3.4851   -3.4728;
    4.2288   -3.3101;
    4.1940   -4.1498];
% n1=20;
% theta1 = linspace(5/4*pi,13/4*pi*(1-1/n1),n1)';
% WAYPOINTS1 = [0,0;cos(theta1)+5,sin(theta1)+5];
% n2 = 20;
% theta2 = linspace(1/4*pi,9/4*pi*(1-1/n2),n2)';
% WAYPOINTS2 = [0,0;cos(theta2)-5,sin(theta2)-5];
% WAYPOINTS=[0,0;WAYPOINTS1;0,0;WAYPOINTS2];
%Using the HILBERT CURVE FOR INITIALIZING WAYPOINTS
% [x1,y1] = hilbert(3);
% waypoints1=20*[0 ,0;x1',y1'];
% [x2,y2] = hilbert(3);
% waypoints2=10*[0 ,0;x2',y2'];

WAYPOINTS=[waypoints1;waypoints2];
f = figure(1);
global MAKE_MOVIE G
MAKE_MOVIE = false;
if MAKE_MOVIE
 MOVIE_NAME = 'WAY POINT CONTROL FOR TWO ROBOT SYSTEM';
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

plotwaypoints(WAYPOINTS,mX,mY,phi,0);
nwaypts = size(WAYPOINTS,1);

indExciting = find(phi>0); %index of every 'exciting point'
for iter = 1:100
    
    for wp=2:nwaypts %iterate through all WAYPOINTS
        H1=[WAYPOINTS(:,1),WAYPOINTS(:,2)];
        
        M_val = 0;
        L_val = 0;
        
        for m = 1:numel(indExciting)  % iterate through every grid point that is 'exciting'
            indx = indExciting(m);
            
            pos = [mX(indx)+cellsz/2,mY(indx)+cellsz/2];  %center of grid cell is [mX,mY] + 1/2[cellsize,cellsize], pos is the center of the interesting gridcell
            
            
            %squared distance between this grid cell and every waypoint
            sumSqDist = sum((repmat(pos,nwaypts,1) - WAYPOINTS).^2,2);
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
            e_val= C_val - [WAYPOINTS(wp,1),WAYPOINTS(wp,2)];
        end
        
        
        Wn=1;
        
        if wp ==nwaypts
            alpha_val=Wn*(WAYPOINTS(wp-1,:)+WAYPOINTS(1,:)-2*WAYPOINTS(wp,:));
        else
            alpha_val=Wn*(WAYPOINTS(wp-1,:)+WAYPOINTS(wp+1,:)-2*WAYPOINTS(wp,:));
        end
        beta_val=M_val+(2*Wn);
        
        % moving towards the centeroid of the voronoi cells
        Ki=1; %potentially -time varying positive definite matrix
        
        
        %         uir((nwaypts+1)/2,:) = [0,0];
        u=Ki.*((M_val.*e_val)+alpha_val)/beta_val; %Control input based on gradient descent
        %apply control input
        deltat=0.1; %time period
        WAYPOINTS(wp,:)=WAYPOINTS(wp,:)+u*deltat; %updating WAYPOINTS
        
        if ( iter<3 && mod(wp,2)) || mod(wp,10)
            plotwaypoints(WAYPOINTS,mX,mY,phi,iter) % calling function
        end
        
    end
    
    
    plotwaypoints(WAYPOINTS,mX,mY,phi,iter) % calling function

    %TODO:  make a flag to decide whether to do ANY plotting
    
end
disp(WAYPOINTS);
close(G.writerObj);
end

function plotwaypoints(WAYPOINTS,mX,mY,phi,iter)
%         pcolor(mX,mY,phi)
imagesc(mX(1,:),mY(:,1),phi)
set(gca,'YDir','normal');
%colormap lines
colormap copper
%         colormap([0,0,0;1,0,0])
hold on
nwp1 = size(10,1);
plot(WAYPOINTS(1:nwp1,1),WAYPOINTS(1:nwp1,2),'go');
plot(WAYPOINTS(nwp1+1:end,1),WAYPOINTS(nwp1+1:end,2),'ro');
axis equal
axis tight
line(WAYPOINTS(1:nwp1+1,1),WAYPOINTS(1:nwp1+1,2),'color','g');
line([WAYPOINTS(nwp1+1:end,1);WAYPOINTS(1,1)],[WAYPOINTS(nwp1+1:end,2);WAYPOINTS(1,2)],'color','r');
%computing the waypoint's Voronoi Partition
voronoi(WAYPOINTS(2:end,1),WAYPOINTS(2:end,2));
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