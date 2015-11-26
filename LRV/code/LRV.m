function[] = LRV()
% This code helps us visualize the "Least Recently Visited algorithm" by
%Sandor P.Fekete et. al, title : "Patrolling a Region with a Structured Swarm 
%of Robots with Limited Capabilities"
clf
clear
clc
figure(1);
axis equal;
tstart(1:11)=tic; %initializing time for each triangle
pts =[30     3; %Initializing points for delaunay's triangulatin
    4    32;
    5    38;
    22    28;
    19     5;
    36    29;
    32     4;
    30     4;
    2    26;
    2    13];
tri = delaunay(pts(:,1),pts(:,2)); %Forming delaunay's triangles
triplot(tri,pts(:,1),pts(:,2),'LineWidth',2); %plotting the triangles
hold on;

x_centroid  = zeros(length(tri),1); %Finding the centroids of the triangles
y_centroid = zeros(length(tri),1); %Useful in forming the Dualgraph
for q =1:length(tri)
    x_centroid(q,1) = mean(pts(tri(q,:),1));
    y_centroid(q,1) = mean(pts(tri(q,:),2));
end
centroid =[x_centroid,y_centroid];
centroid_sort = sortrows(centroid);%sort in assending order for drawing dual graph
% disp(centroid_sort);
plot(centroid(:,1),centroid(:,2),'rO','LineWidth',2);  
    
dualgraph =[1,3; %Assigning the neighbours for each triangle
    1,4;
    2,3;
    2,5;
    3,4;
    4,6;
    5,7;
    6,9;
    7,8;
    7,9;
    8,10;
    9,11;
    10,11];
p=[centroid_sort(dualgraph(1:end,1),1),centroid_sort(dualgraph(1:end,2),1),centroid_sort(dualgraph(1:end,1),2),centroid_sort(dualgraph(1:end,2),2)];
% disp(p);
for i =1:length(p)
    plot(p(i,1:2),p(i,3:4),'k-','LineWidth',2);
    hold on
end
r =1;%Assign initial position of robot on the map(Assuming the robot is placed only on the centroids of the triangle)
figure(1);
hLine = line('XData',centroid_sort(r,1), 'YData',centroid_sort(r,2), 'Color','g', ...
    'Marker','s', 'MarkerSize',10, 'LineWidth',2,'MarkerFaceColor','r');

for L=1:200
 [x,y,A,B] = NextMove(dualgraph,centroid_sort,r,tstart);
 MoveRobot(x,y,hLine); 
 r=A;
 tstart=B;
end
end

function[] = MoveRobot(x,y,hLine)
set(0,'RecursionLimit',1500);
pause(1);
set(hLine, 'XData',x, 'YData',y); % Used this for robot motion
end

function[x,y,r,tstart] = NextMove(dualgraph,centroid_sort,r,tstart)
z=[];
for q=1:length(dualgraph)
if r==dualgraph(q,1) % I find the neighbours of the triangle at which the robot is present
    z=[z;dualgraph(q,2)];
elseif r==dualgraph(q,2)    
   z=[z;dualgraph(q,1)];
end
end
disp 'List of neighbours at a particular centroid';
disp(z);
func=[];
for w=1:length(z)
func=[func;toc(tstart(1,z(w,1)))]; % We choose the first triangle
end
disp 'Function of time of the neighbours when robot is at a given centroid';
disp(func);% time in all the neighbouring triangles
   [~,no] = max(func); % We find the maximum time among the neighbours
disp 'Max item in the time function';
disp(no);   
x=centroid_sort(z(no,1),1); %New location where the robot needs to move
y=centroid_sort(z(no,1),2); 
r=z(no,1); %New starting point
disp 'New sensor triangle centroid location number';
disp(r);
tstart(1,r)=tic; %We reset the time in this sensor node
disp 'Time in the sensor triangle centroid that was reset';
disp(toc(tstart(1,r)));
disp 'Time in the first sensor triangle centroid';
disp(toc(tstart(1,1)));
end


