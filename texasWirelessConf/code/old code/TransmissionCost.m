function [O] = TransmissionCost(h, xs, ys, pow, xr, yr)
%  h is height of robot
% [xs,ys] center of sensor nodes, 
% [xr,yr] are grid points
%Defining a function for the intresting points
% We would use it in waypointControlForPathPlaning as a function that 
% generates intresting points for robots
O = 1./(((h)^(2))+((xs-xr).^(2))+((ys-yr).^(2))).^((pow)/(2));
O =   1* ( ((xs-xr).^2+(ys-yr).^2)<3 ); %Binary cost -- is it better?

