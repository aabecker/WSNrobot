sensors=[0,0;-2,2;2,2;-2,-2;2,-2;-6,6;-8,8;-4,8;-8,4;-4,4;6,-6;8,-8;4,-8;8,-4;4,-4;-10,-10;10,10;-8,-10;-10,-8;8,10;10,8];
plot(sensors(1:end,1),sensors(1:end,2),'ro');
axis([-11 11 -11 11]); 
 title 'Position of Sensor Nodes';
    xlabel 'X-axis(m)';
    ylabel 'Y-axis(m)';
    
    