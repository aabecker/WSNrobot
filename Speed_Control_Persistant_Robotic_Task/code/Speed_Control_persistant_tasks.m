%Simulation adapted from:-
%IEEE Transactions on Robotics, VOL.28, NO.2, April 2012
%Title: Persistant Robotic Tasks: Monitoring and Sweeping in Changing Environments
%Authors: Stephen L.Smith, Mac Schwager, Daniela Rus

%Adapted by: Srikanth.K.V.S and Dr.Aaron.T.Becker
%University of Houston, Texas, U.S.A

szInteresting = 200;
phi = zeros(szInteresting,szInteresting);
%these are interesting!!!
[mX,mY] = meshgrid(linspace(-15,15,szInteresting),linspace(-15,15,szInteresting)); %making the grid
cellsz = mY(2) - mY(1);
%Location of the Sensor nodes
sensors=[-10,-10;-10,-8;-8,-10;-8,4;-8,8;-6,6;-4,4;-4,8;-2,-2;-2,2;0,0;2,-2;2,2;4,-8;4,-4;6,-6;8,-8;8,-4;8,10;10,8;10,10];
grid on;
sink=11;


for m=2:length(sensors)
   
  Figure=  line([sensors(m-1,1) sensors(m,1)],[sensors(m-1,2) sensors(m,2)],'Marker','O','LineStyle','-');
    hold on;
    if m==length(sensors)
      Figure=  line([sensors(m,1);sensors(1,1)],[sensors(m,2);sensors(1,2)],'Marker','O','LineStyle','-');
        hold on;
    end    
end

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

 