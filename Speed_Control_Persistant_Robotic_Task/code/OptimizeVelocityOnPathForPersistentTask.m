function OptimizeVelocityOnPathForPersistentTask()
    %Location of the Sensor nodes
    sensors= [0,2
              0,-2];
    ax = axes('XLim',[-3 3],'YLim',[-3 3]);
    hold on;
    r=2;
    circle(0,0,r ); 
    for p=1:length(sensors)
    line(sensors(p,1),sensors(p,2),'Marker','s','markersize',14,'markeredgecolor','k','linewidth',1, 'markerfacecolor',[1,0,0]);
    hold on;
    end
vel=5; % this also influences the number of times it travels along the same path
grid on
t = hgtransform('Parent',ax);
line(0,0,'Marker','O','markersize',30,'markeredgecolor','m','linewidth',2, 'markerfacecolor',[0,0,1],'Parent',t);
hold on;
n=0.1; % accuracy, step size

for i=0:n:2*pi
    M = eye(4);
    M(1,4) = 2*cos(vel*i); %x-position
    M(2,4) = 2*sin(vel*i); %y-position
    Rz = makehgtform('zrotate',-vel*i);
    set(t,'Matrix',M*Rz);
pause(1)
end
gamma=Calc_gamma(r,n);
disp(gamma);
end

function gamma= Calc_gamma(r,y)
gamma=[];
for i=0:y:2*pi
tot_dist=2*pi*r;
theta=i/(2*pi);
dist_traversed=(theta*tot_dist);
gamma=[gamma,dist_traversed/tot_dist];
end
gamma=[1,gamma];
end
