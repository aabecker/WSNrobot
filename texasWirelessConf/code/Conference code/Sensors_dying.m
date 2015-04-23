function Sensors_dying(sensors,sink)
%make out interesting function
%Input the 'sensors' which give the location of the sensor nodes and the size
%of the matrix is (n,2) where 'n' is the number of nodes
%Indicate the position of the sink node from the list of sensors

global MAKE_MOVIE G
G.fig = figure(1);
clf
G.init = false;
MAKE_MOVIE = true;
if MAKE_MOVIE
    MOVIE_NAME = 'Sensor Network Dying'; %#ok<*UNRCH>
    
    clf
    set(G.fig,'Units','normalized','outerposition',[0 0 1 1],'NumberTitle','off','MenuBar','none','color','w');
    G.writerObj = VideoWriter(MOVIE_NAME,'MPEG-4');%http://www.mathworks.com/help/matlab/ref/videowriterclass.html
    set(G.writerObj,'Quality',100);
    open(G.writerObj);
end

set(G.fig,'Name','Dying_Sensor_Network');

szInteresting = 200;
phi = zeros(szInteresting,szInteresting);
%these are interesting!!!
[mX,mY] = meshgrid(linspace(-15,15,szInteresting),linspace(-15,15,szInteresting)); %making the grid
cellsz = mY(2) - mY(1);
%Location of the Sensor nodes
sensors=[-10,-10;-10,-8;-8,-10;-8,4;-8,8;-6,6;-4,4;-4,8;-2,-2;-2,2;0,0;2,-2;2,2;4,-8;4,-4;6,-6;8,-8;8,-4;8,10;10,8;10,10];
Sensorsize=size(sensors(:,1));
sink=11;

for m = 1:size(sensors,1)
    h = 1.25;
    phi=phi+TransmissionCostCalc(h,sensors(m,1),sensors(m,2),6, mX, mY);
end
imagesc(mX(1,:),mY(:,1),phi)
set(gca,'YDir','normal');
colormap([1,1,1;0,0,1]);
% grid off;
hold on;
% PlotBatteryPositions(sensors,0,1,0);
% for creating the battery location on the map
A=sensors+ones(size(sensors));
for t=1:length(sensors)
    figure1(t)= rectangle('Position',[A(t,1),A(t,2),0.7,0.3],...
        'Curvature',[0,0],...
        'FaceColor',[0 1 0]);
    daspect([1,1,1]);
    xlabel ('X-axis(m)','FontSize',14,...
       'FontWeight','bold');
    ylabel ('Y-axis(m)','FontSize',14,...
       'FontWeight','bold');
    hold on;
end
W = CalculateDistSensorNodes(sensors);%Calculate distance between each sensor
% Weight matrix for the minimum spanning tree it is a DISTANCE matrix which indicates the distance
% disp(W);

X=ones(21); % Adjecency Matrix indicates the connectivity between sensor nodes
X = X-diag(diag(X));
[a,b,c]= kruskal(X,W);

H_links=PlotMinimumSpanningTree (sensors,b);
PowerLoss= [0.06 ,0.01 ,0.025]; % PowerLoss (mA) = [sensing listening transmitting] SCALED by a factor of 10
%Considering the battery has 10Ah and assuming the the discharge rate
%doesnot affect the Ampere hour rating of the battery

Y=DirectedMinSpanningTree(b,sink); %function gives the directed spanning graph
v=W.*c;  % gives the distance betwwen the connected nodes in the spanning tree

Sensorloss=ones(size(sensors(:,1)));
 Working_sensors=ones(Sensorsize(1,1),1);
 Z=ones(Sensorsize(1,1));
 Z = Z-diag(diag(Z));
 Dist=[];
 W=CalculateDistSensorNodes(sensors);
for n=1:50
title(['Iteration','', num2str(n)]);
p1=plot(nan,nan,'s','markersize',16,'markeredgecolor','k','linewidth',2, 'markerfacecolor',[0,1,0]);
hold on;
p2=plot(nan,nan,'s','markersize',16,'markeredgecolor','k','linewidth',2, 'markerfacecolor',[1,1,0]);
hold on;
p3=plot(nan,nan,'s','markersize',16,'markeredgecolor','k','linewidth',2, 'markerfacecolor',[1,0.5,0]);
hold on;
p4=plot(nan,nan,'s','markersize',16,'markeredgecolor','k','linewidth',2, 'markerfacecolor',[1,0,0]);
hold on;
p5=plot(nan,nan,'-m','linewidth',2);
hold on;
legend([p1,p2,p3,p4,p5],{'100%-70% power','70%-30% power','30%-5% power','below 5% power','Min. Spanning Tree'},'location','eastOutside')
  if Sensorloss(sink,1)>0.1  
   
    for p=1:Sensorsize(1,1)
        %      DepletionOfCharge( p,Sensorloss,sensors,h_links,PowerLoss,n,Y,v)
        %Let the charge in sensors deplete slowly
        if Sensorloss(p,1)<=0.1
       Working_sensors(p,1)=0;
%        Q=1./Working_sensors;     
Z(p,:)=0;Z(:,p)=0;
W(p,:)=Inf;W(:,p)=Inf;
           %disp(W);
            [a,b,c]= kruskal(Z,W);
            %disp(c);
            v=W.*c;
            delete(H_links);
            [H_links]=PlotMinimumSpanningTree(sensors,b);
            hold on;
            %    end
            % end
            
elseif Sensorloss(p,1)>0.1
            R=rem(n,1); % The sensor senses it's surrounding once every 5 cycles
            if R==0
                K(p)=poissrnd(1);
                Sensorloss(p,1)= Sensorloss(p,1) - K(p)*(PowerLoss(1,1));
                if Sensorloss(p,1) < 0.1
                    figure1(p).FaceColor = [1, 0, 0];
                   Sensorloss(p,1)=0.1;
                else
                    if Sensorloss(p,1) < 0.7 && Sensorloss(p,1) > 0.3
                        figure1(p).FaceColor = [1, 1, 0];
                    elseif Sensorloss(p,1) < 0.3
                        figure1(p).FaceColor = [1,0.5, 0];
                    else
                        figure1(p).FaceColor = [0, 1, 0];
                    end
                end
                pause(0.1);
            end
            for n1=1:length(Y)
                if p==Y(n1,1)
                    if Sensorloss(Y(n1,2))~=0.1
                        Sensorloss(p,1) = Sensorloss(p,1) - K(p)*(PowerLoss(1,3)*(v(Y(n1,1),Y(n1,2))));
                        if Sensorloss(p,1) < 0.1
                            figure1(p).FaceColor = [1, 0, 0];
                           Sensorloss(p,1)=0.1;
                        else
                            if Sensorloss(p,1) < 0.7 && Sensorloss(p,1) > 0.3
                                figure1(p).FaceColor = [1, 1, 0];
                            elseif Sensorloss(p,1) < 0.3
                                figure1(p).FaceColor = [1, 0.5, 0];
                            else
                                figure1(p).FaceColor = [0, 1, 0];
                            end
                        end
                    end
                    pause(0.1);
                    
                elseif p==Y(n1,2)
                    
                    Sensorloss(p,1) = Sensorloss(p,1) - PowerLoss(1,2);
                    if Sensorloss(p,1) < 0.1
                        figure1(p).FaceColor = [1, 0, 0];
                     Sensorloss(p,1)=0.1;
                    else
                        if Sensorloss(p,1) < 0.7 && Sensorloss(p,1) < 0.3
                            figure1(p).FaceColor = [1, 1, 0];
                        elseif Sensorloss(p,1) < 0.3
                            figure1(p).FaceColor = [1, 0.5, 0];
                        else
                            figure1(p).FaceColor = [0, 1, 0];
                        end
                    end
                    pause(0.1);
                end
            end
            pause(0.1);
        end
    end
  end
   disp 'loss';
    disp(n);
    disp (Sensorloss);
 if MAKE_MOVIE
    close(G.writerObj);
end   
end
end

function[DistMatx] = CalculateDistSensorNodes(sensors)
% calculating distance between each sensor
Dist=[];
for q=1:length(sensors)
    for var1=1:length(sensors)
        O = (((sensors(q,1)-sensors(var1,1)).^(2))+((sensors(q,2)-sensors(var1,2)).^(2)))^(0.5);
        Dist=[Dist;O];
    end
end
L=0:length(sensors):(length(sensors)*length(sensors));
Dist_var1=[];
w= [Dist(1:L(2),1)'];
for q=2:length(sensors)
    Dist_var1=[Dist_var1;Dist(L(q)+1:L(q+1),1)'];
end
DistMatx=[w;Dist_var1];
end

% function PlotBatteryPositions(sensors,x,y,z)
% % for creating the battery location on the map
% A=sensors+ones(size(sensors));
% for t=1:length(sensors)
%     figure1(t)= rectangle('Position',[A(t,1),A(t,2),0.7,0.3],...
%         'Curvature',[0,0],...
%         'FaceColor',[x y z]);
%     daspect([1,1,1]);
%       xlabel 'X-axis(m)';
%       ylabel 'Y-axis(m)';
% hold on;
% end
% end
function [h_links]=PlotMinimumSpanningTree (sensors,b)
%Plots the minimum spanning tree on the map
h_links=zeros(length(b),1);
for u=1:length(b)
    x=[sensors(b(u,1),1) sensors(b(u,2),1)];
    y=[sensors(b(u,1),2) sensors(b(u,2),2)];
    h_links(u)= plot(x,y,'m -');
     set(h_links(u), 'linewidth',2);
    hold on;
end
end
function makemovie()
global MAKE_MOVIE G
if MAKE_MOVIE
    % (for each frame)
    figure(G.fig)
    set(gcf,'renderer','painters')  %use these settings for final
    tfig = myaa(3);
    F = getframe;
    writeVideo(G.writerObj,F.cdata);
    close(tfig)
end
end
