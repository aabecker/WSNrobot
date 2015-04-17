function phi_plotting= inputs(sensors)
sensors=[0,0;-2,2;2,2;-2,-2;2,-2;-6,6;-8,8;-4,8;-8,4;-4,4;6,-6;8,-8;4,-8;8,-4;4,-4;-10,-10;10,10;-8,-10;-10,-8;8,10;10,8];
plot(sensors(1:end,1),sensors(1:end,2),'ro');
axis([-11 11 -11 11]); 
 title 'Position of Sensor Nodes';
    xlabel 'X-axis(m)';
    ylabel 'Y-axis(m)';
    
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