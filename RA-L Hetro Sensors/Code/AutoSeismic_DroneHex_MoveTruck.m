function [] = AutoSeismic_DroneHex_MoveTruck(x,y,T,hex)
% simulates a seismic survey with
% X-length in m, Y-length in m, T = Initial HomeBase location in [x,y];
% hex: the number of hexapods
%  Srikanth KVS and Aaron Becker
%
%  I want two variables:  State (S) and Graphics (G)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%
if nargin <1
    x = 100;
    y = 200;
    T = [0,y/2];
    hex = 20; %total number of Hexapod walkers
    drones = 50; %total number of Drones
    darts = 500; %total number of Darts
    drone_cap = 4; %number of darts a drone can hold
    people = 0; %total number of human workers
    people_cap = 0; %number of geophones a human can hold
end
%constants
S.T = T; S.x = x; S.y = y; S.hex = hex; S.drones = drones; S.darts = darts; S.drone_cap = drone_cap; S.people = people;S.people_cap = people_cap;
dt = 1; % delta T
tic
S = Init_State(S);
G = Init_Graphics(S); %draw everything for first time
for T = 1:dt:10000000 % Main_loop
    if mod(T,1000) == 0
        display(['T = ',num2str(T)]) %shows ocmputer is still running
    end
    S = Update_State(S,dt);
    G = Update_Graphics(S,G,T);
    if isFinished(S)
        break
    end
end
title(['finished in T = ',num2str(T),' seconds'])
display(['T = ',num2str(T),', ',num2str(S.numShots),' shots'])
display([ x,y,hex ,drones,darts,drone_cap,people,people_cap,T])
toc
end

function S = Init_State(S)
% constants
gridSpacing = 10; %meters between grid points
S.minSensorsForShot = min(10, S.hex+S.darts*(S.drones>0)); %minimum number of ready sensors before a shot is taken.
S.numShots = 0;

%initialize survey points
[xg,yg] = meshgrid(0:gridSpacing:S.x,0:gridSpacing:S.y);
S.surveyPtsX = xg(:);
S.surveyPtsY = yg(:);
S.surveyPtsState = zeros(size(S.surveyPtsX));
%state: 0 no measurement, 1 assigned, 3 measured

%Initialize truck
S.Tstate = 2; % 0 - unassigned 1 - assigned 2-ready (at position) 3 - finished
S.Tpos = S.T;
S.Tstart = S.T;
S.Tgoal = S.T;
S.Tvel = 1;

%initialize hex
S.Hstate = zeros(S.hex,1); % 0 - unassigned 1 - assigned 2-ready (at position)
S.Hpos = repmat( S.T, S.hex,1);
S.Hstart = S.Hpos;
S.Hgoal = S.Hpos;
S.Hvel = 0.20;
S.Hsurveypt = zeros(S.hex,1); %ID of the survey point assigned to this robot
%initilize drones
S.Qstate = zeros(S.drones,1); % 0- unassigned 1- assigned
S.Qdeploying = zeros(S.drones,1); % 0- retrieve 1- deploy mode
S.Qpos = repmat( S.T, S.drones,1);
S.Qdarts = zeros(S.drones,S.drone_cap); %IDs of darts on drone (0 is no dart)
S.Qdartpickup = zeros(S.drones,1); %ID of dart to pick up.
S.Qdartscap = zeros(S.drones,1); %how many darts on board
S.Qstart = S.Qpos;
S.Qgoal = S.Qpos;
S.Qvel = 0.8;
S.Qsurveypt = zeros(S.drones,1);%ID of the survey point assigned to this robot (do we need this?)
%initialize darts
S.Dstate = zeros(S.darts,1);% 0- unassigned 1- assigned 2-ready (at position)
S.Dpos = repmat( S.T, S.darts,1);
S.Dsurveypt = zeros(S.darts,1);

end

function isDone = isFinished(S)
isDone = (numel(S.surveyPtsState) == sum(S.surveyPtsState == 3));
end

function S=Update_State(S,dt)
%decide whether to do shot test (if so, update states)
S = shotTest(S);
% move agents, update agent & survey point state
S = moveHexPod(S,dt);
S = moveDrone(S,dt);
%assign agents
S = assignHexPod(S);
S = assignDrone(S);
S = moveTruck(S,dt);

end
function S = shotTest(S)
%state: 0 no measurement, 1 assigned, 2 ready for measurement, 3 measured,
readyPts = sum(S.surveyPtsState == 2);
if readyPts >= S.minSensorsForShot ||  sum(S.surveyPtsState == 0)+sum(S.surveyPtsState == 1)==0
    S.numShots = S.numShots+1;
    S.surveyPtsState(S.surveyPtsState == 2) = 3; %mark as measured
    S.Hstate(S.Hstate == 2) = 0; %hexpods can move
    S.Dstate(S.Dstate == 2) = 0; % darts can be moved
end
end


function S = moveHexPod(S,dt) %move Hex toward goal position
% 0 - unassigned 1 - assigned 2-ready (at position)
for k = 1:S.hex
    
    dxy = sqrt(sum((S.Hgoal(k,:) - S.Hpos(k,:)).^2));
    if S.Hstate(k) == 1
        if dxy > S.Hvel*dt
            S.Hpos(k,:) = S.Hpos(k,:)+S.Hvel*dt*(S.Hgoal(k,:) - S.Hpos(k,:))/dxy;
        else
            S.Hpos(k,:) = S.Hgoal(k,:);
            S.Hstate(k) = 2;  %hexpod is ready for test here
            S.surveyPtsState(S.Hsurveypt(k)) = 2; %mark survey ready for test (hexapod)
        end
    end
end
end

function S = assignHexPod(S)
% for each unassigned hexpod, assign to the nearest unassigned survey
% point
for k = 1:S.hex % 0 - unassigned 1 - assigned 2-ready (at position)
    % if unassigned, move to the next (doesn't get unassigned until shot
    % is taken)
%      if numind>0
%                 dist = sqrt(sum((repmat( S.Qpos(k,:),numind,1) - [S.surveyPtsX(ind),S.surveyPtsY(ind)]).^2,2))+...
%                     sqrt(sum((repmat( S.Tpos,numind,1) - [S.surveyPtsX(ind),S.surveyPtsY(ind)]).^2,2));
%                 [~,i] = min(dist);
%                 c = find( S.surveyPtsState == 0, i,'first');
%                 minInd =c(end);
%                 
%                 S.Qgoal(k,:) = [S.surveyPtsX(minInd),S.surveyPtsY(minInd)];
%                 S.surveyPtsState(minInd) = 1; %assigned
%                 S.Qstate(k) = 1; %assigned
%                 S.Qstart(k,:) = S.Qpos(k,:);
%                 S.Qsurveypt(k) = minInd;
%             end
    if S.Hstate(k) == 0
       
%         for m = 1:numel(S.surveyPtsX)
%             if S.surveyPtsState(m) == 0 %no measurement & unassigned
%                 dist = sqrt(sum((S.Hpos(k,:) - [S.surveyPtsX(m),S.surveyPtsY(m)]).^2))+...
%                     sqrt(sum((S.Tpos - [S.surveyPtsX(m),S.surveyPtsY(m)]).^2));
%                 if dist<minDist
%                     minInd = m;
%                     minDist = dist;
%                 end
%             end
%         end
ind = (S.surveyPtsState == 0);
numind = sum(ind);
    if numind>0
                dist = sqrt(sum((repmat( S.Hpos(k,:),numind,1) - [S.surveyPtsX(ind),S.surveyPtsY(ind)]).^2,2))+...
                    sqrt(sum((repmat( S.Tpos,numind,1) - [S.surveyPtsX(ind),S.surveyPtsY(ind)]).^2,2));
                [~,i] = min(dist);
                c = find( S.surveyPtsState == 0, i,'first');
                minInd =c(end);
                
        if ~isnan(minInd)
            S.Hgoal(k,:) = [S.surveyPtsX(minInd),S.surveyPtsY(minInd)];
            S.surveyPtsState(minInd) = 1; %assigned
            S.Hstate(k) = 1; %assigned
            S.Hstart(k,:) = S.Hpos(k,:);
            S.Hsurveypt(k) = minInd;
        end
    end
   end
end
end

% function S = assignHexPod(S)
% % for each unassigned hexpod, assign to the nearest unassigned survey
% % point
% for k = 1:S.hex % 0 - unassigned 1 - assigned 2-ready (at position)
%     % if unassigned, move to the next (doesn't get unassigned until shot
%     % is taken)
%     if S.Hstate(k) == 0
%         minDist = 10*S.x*S.y; %a very large number
%         minInd = NaN;
%         for m = 1:numel(S.surveyPtsX)
%             if S.surveyPtsState(m) == 0 %no measurement & unassigned
%                 dist = sqrt(sum((S.Hpos(k,:) - [S.surveyPtsX(m),S.surveyPtsY(m)]).^2))+...
%                     sqrt(sum((S.Tpos - [S.surveyPtsX(m),S.surveyPtsY(m)]).^2));
%                 if dist<minDist
%                     minInd = m;
%                     minDist = dist;
%                 end
%             end
%         end
%         if ~isnan(minInd)
%             S.Hgoal(k,:) = [S.surveyPtsX(minInd),S.surveyPtsY(minInd)];
%             S.surveyPtsState(minInd) = 1; %assigned
%             S.Hstate(k) = 1; %assigned
%             S.Hstart(k,:) = S.Hpos(k,:);
%             S.Hsurveypt(k) = minInd;
%         end
%     end
% end
% end

function S = moveDrone(S,dt) %move Drone (and onboard darts) toward goal position
%S.Qstate  0- unassigned 1- assigned
%S.Qdeploying  0- retrieve 1- deploy mode
%S.Qdarts IDs of darts on drone (0 is no dart)
for k = 1:S.drones
    if S.Qdeploying(k) == 1 && S.Qstate(k) == 1 %deploy darts
        dxy = sqrt(sum((S.Qgoal(k,:) - S.Qpos(k,:)).^2));
        if dxy > S.Qvel*dt
            S.Qpos(k,:) = S.Qpos(k,:)+S.Qvel*dt*(S.Qgoal(k,:) - S.Qpos(k,:))/dxy;
        else
            S.Qpos(k,:) = S.Qgoal(k,:);
            S.Qstate(k) = 0;  %Unassigned so that the drone can go to it's next location for deployment
            dartInd = find(S.Qdarts(k,:) >0,1,'first');
            dartID = S.Qdarts(k,dartInd);
            S.Qdarts(k,dartInd) = 0;  %remove dart from drone
            S.Dstate(dartID) =  2;  % dart is ready for measurement
            S.Dpos(dartID,:) = S.Qgoal(k,:); % dart position is the survey point
            S.Dsurveypt(dartID) = S.Qsurveypt(k); %assign survey point to dart
            S.surveyPtsState(S.Dsurveypt(dartID)) = 2; %mark survey ready for test (dart)
            
            if sum(S.Qdarts(k,:)>0) == 0 % if number of onboard darts = 0, time to retrieve
                S.Qdeploying(k) = 0;
            end
        end
    elseif S.Qdeploying(k) == 0 && S.Qstate(k) == 1 %retrieving darts
        dxy = sqrt(sum((S.Qgoal(k,:) - S.Qpos(k,:)).^2));
        if dxy > S.Qvel*dt
            S.Qpos(k,:) = S.Qpos(k,:)+S.Qvel*dt*(S.Qgoal(k,:) - S.Qpos(k,:))/dxy;
        else
            S.Qpos(k,:) = S.Qgoal(k,:);
            S.Qstate(k) = 0;  %Unassigned so that the drone can go to it's next location for deployment
            emptyInd = find(S.Qdarts(k,:) == 0,1,'first'); %find empty slot on quad
            S.Qdarts(k,emptyInd) = S.Qdartpickup(k);  %add dart to drone
            S.Dsurveypt(S.Qdartpickup(k)) = 0; %unassign dart's survey point
            S.Qdartpickup(k) = 0; %dart is picked up
            if sum(S.Qdarts(k,:)>0) == S.drone_cap % if full of darts, time to deploy
                S.Qdeploying(k) = 1;
            end
        end
    end
    % update position of all onboard darts
    count = 1;
    for m = 1:numel(S.Qdarts(k,:))
        if S.Qdarts(k,m) > 0
            S.Dpos(S.Qdarts(k,m),:) = S.Qpos(k,:)+[count,-2];
            count = count+1;
        end
    end
end
end


function S = assignDrone(S)
% for each unassigned drone:
% if in deploy mode, assign to the nearest unassigned survey point
% if in retrieve mode, assign to nearest ready dart
for k = 1:S.drones  % 0- unassigned 1- assigned
    if  S.Qstate(k) == 0 %only assign if unassigned
        if S.Qdeploying(k) == 0 % need to retrieve darts -- so find the closest dart
            minDist = 10*S.x*S.y; %a very large number
            minInd = NaN;
            for m = 1:S.darts
                if S.Dstate(m) == 0 %no measurement & unassigned
                    dist = sqrt(sum((S.Qpos(k,:) - S.Dpos(m,:)).^2));
                    if dist<minDist
                        minInd = m;
                        minDist = dist;
                    end
                end
            end
            if ~isnan(minInd)
                S.Qgoal(k,:) = S.Dpos(minInd,:);
                S.Qstate(k) = 1; %assigned
                S.Qstart(k,:) = S.Qpos(k,:);
                S.Dstate(minInd) = 1; %dart is assigned
                S.Qdartpickup(k) = minInd;
                S.Dsurveypt(k) = 0; %no survey point assigned
            end
            
        else % need to deploy darts -- so find the closest survey point
            ind = (S.surveyPtsState == 0);
            numind = sum(ind);
            if numind>0
                dist = sqrt(sum((repmat( S.Qpos(k,:),numind,1) - [S.surveyPtsX(ind),S.surveyPtsY(ind)]).^2,2))+...
                    sqrt(sum((repmat( S.Tpos,numind,1) - [S.surveyPtsX(ind),S.surveyPtsY(ind)]).^2,2));
                [~,i] = min(dist);
                c = find( S.surveyPtsState == 0, i,'first');
                minInd =c(end);
                
                S.Qgoal(k,:) = [S.surveyPtsX(minInd),S.surveyPtsY(minInd)];
                S.surveyPtsState(minInd) = 1; %assigned
                S.Qstate(k) = 1; %assigned
                S.Qstart(k,:) = S.Qpos(k,:);
                S.Qsurveypt(k) = minInd;
            end
        end
    end
end
end

function S = moveTruck(S,dt) %move Truck (and onboard darts) in +x
% find the survey point with lowest x value.
xMin = min(S.surveyPtsX(S.surveyPtsState < 3));
if ~isempty(xMin)
    S.Tgoal(1) = xMin;
end
oldPos = S.Tpos;
 dxy = sqrt(sum((S.Tgoal - S.Tpos).^2));
        if dxy > S.Tvel*dt
            S.Tpos = S.Tpos+S.Tvel*dt*(S.Tgoal - S.Tpos)/dxy;
        else
            S.Tpos = S.Tgoal;
        end
    % update position of all onboard darts
    for m = 1:numel(S.Dstate)
        if S.Dstate(m) == 0 &&  S.Dpos(m,1) == oldPos(1) &&S.Dpos(m,2) == oldPos(2)
            S.Dpos(m,:) = S.Tpos;
        end
    end
end
function G = Init_Graphics(S)
figure(1);
clf;
set(gca,'Xlim',[0,S.x],'Ylim',[0,S.y]);
%draw survey points
G.surveyPts = scatter(S.surveyPtsX,S.surveyPtsY,[],S.surveyPtsState);
hold on
axis equal
grid on;

%draw truck
G.tPathG = line([S.Tstart(1),S.Tgoal(1)],[S.Tstart(2),S.Tgoal(2)],'color' ,'b');
G.tPathP = line([S.Tpos(1),S.Tgoal(1)],[S.Tpos(2),S.Tgoal(2)],'color' ,'r');
tpts = truckPts(S.Tpos(1),S.Tpos(2),1);
G.truck = fill(tpts(:,1),tpts(:,2),'b');

%draw darts
G.da = ones(S.darts,1);
for k = 1:S.darts     %draw the darts
    dapts = daPts(S.Dpos(k,1),S.Dpos(k,2),1);
    G.da(k) = fill(dapts(:,1),dapts(:,2),'y');
end

%draw hexpods
G.h = ones(S.hex,1);
G.hPathG = ones(S.hex,1);
G.hPathP = ones(S.hex,1);
for k = 1:S.hex % draw desired paths
    G.hPathG(k) = line([S.Hstart(k,1),S.Hgoal(k,1)],[S.Hstart(k,2),S.Hgoal(k,2)],'color' ,'b');
end
for k = 1:S.hex    %draw current path
    G.hPathP(k) = line([S.Hpos(k,1),S.Hgoal(k,1)],[S.Hpos(k,2),S.Hgoal(k,2)],'color' ,'r');
end
for k = 1:S.hex     %draw the hexpods
    hpts = hexPts(S.Hpos(k,1),S.Hpos(k,2),1);
    G.h(k) = fill(hpts(:,1),hpts(:,2),'r');
end
%draw quads
G.d = ones(S.drones,1);
G.DPathG = ones(S.drones,1);
G.DPathP = ones(S.drones,1);
for k = 1:S.drones % draw desired paths
    G.DPathG(k) = line([S.Qstart(k,1),S.Qgoal(k,1)],[S.Qstart(k,2),S.Qgoal(k,2)],'color' ,'g');
end
for k = 1:S.drones    %draw current path
    G.DPathP(k) = line([S.Qpos(k,1),S.Qgoal(k,1)],[S.Qpos(k,2),S.Qgoal(k,2)],'color' ,'m');
end
for k = 1:S.drones    %draw the drones
    dpts = dPts(S.Qpos(k,1),S.Qpos(k,2),1);
    G.d(k) = fill(dpts(:,1),dpts(:,2),'k');
end

%draw people
hold off
drawnow;
end % end init_graphics

function G = Update_Graphics(S,G,T)
%update survey points
set(G.surveyPts,'CData',S.surveyPtsState);
%update truck
tpts = truckPts(S.Tpos(1),S.Tpos(2),1); set(G.truck, 'XData',tpts(:,1) ,'YData',tpts(:,2)) ;

%draw hexpods
for k = 1:S.hex
    hpts = hexPts(S.Hpos(k,1),S.Hpos(k,2),1);
    set( G.h(k), 'XData',hpts(:,1) ,'YData',hpts(:,2));
    set( G.hPathG(k), 'XData',[S.Hstart(k,1),S.Hgoal(k,1)],'YData',[S.Hstart(k,2),S.Hgoal(k,2)]);
    set( G.hPathP(k), 'XData',[S.Hpos(k,1),S.Hgoal(k,1)],'YData',[S.Hpos(k,2),S.Hgoal(k,2)]);
end
%draw quads
for k = 1:S.drones
    dpts = dPts(S.Qpos(k,1),S.Qpos(k,2),1);
    set( G.d(k), 'XData',dpts(:,1) ,'YData',dpts(:,2));
    set( G.DPathG(k), 'XData',[S.Qstart(k,1),S.Qgoal(k,1)],'YData',[S.Qstart(k,2),S.Qgoal(k,2)]);
    set( G.DPathP(k), 'XData',[S.Qpos(k,1),S.Qgoal(k,1)],'YData',[S.Qpos(k,2),S.Qgoal(k,2)]);
end
%draw darts
for k = 1:S.darts
    dapts = daPts(S.Dpos(k,1),S.Dpos(k,2),1);
    set( G.da(k), 'XData',dapts(:,1) ,'YData',dapts(:,2));
end
%update truck
set(G.tPathG, 'XData',[S.Tstart(1),S.Tgoal(1)],'YData',[S.Tstart(2),S.Tgoal(2)]);
set(G.tPathP, 'XData',[S.Tpos(1),S.Tgoal(1)],'YData',[S.Tpos(2),S.Tgoal(2)]);
tpts = truckPts(S.Tpos(1),S.Tpos(2),1); set(G.truck, 'XData',tpts(:,1) ,'YData',tpts(:,2)) ;
%draw people

%update title
title(['T = ',num2str(T),', ',num2str(sum(S.surveyPtsState == 2)),' pts ready, ',num2str(S.numShots),' shots'])
drawnow;
end % end update graphics

function pts = hexPts(xOff,yOff,sc) %draw a hexagon
th = 0:pi/3:2*pi;
pts = sc*[xOff+cos(th)',yOff+sin(th)'];
end

function pts = dPts(xOff,yOff,sc) %draw a drone
th = 0:pi/2:2*pi;
pts = sc*[xOff+cos(th)',yOff+sin(th)'];
end

function pts = daPts(xOff,yOff,sc) %draw a dart
th = -pi/2+(0:2*pi/3:2*pi);
pts = sc*[xOff+cos(th)',yOff+sin(th)'];
end

function pts = truckPts(xOff,yOff,sc)
pts = [-6.1757   -1.5258
    0.6896   -1.5258
    0.6979   -3.5692
    2.8979   -3.5692
    3.7789   -1.9548
    6.2676   -1.6545
    6.2979    0.5308
    5.0979    0.6308
    4.6371   -0.2814
    3.4357   -0.2814
    3.0979    0.5308
    3.6073    0.5338
    4.0793    0.1476
    4.6800    0.4909
    4.6979    1.0308
    4.0793    1.3920
    3.5215    1.0487
    3.5215    0.5338
    3.0495    0.5338
    -3.0863    0.5338
    -3.6441   -0.3243
    -4.8026   -0.3243
    -5.3175    0.4480
    -4.6739    0.4051
    -4.1590    0.1905
    -3.6441    0.3622
    -3.5154    0.8342
    -3.8157    1.3491
    -4.3735    1.3491
    -4.7597    0.8771
    -4.6739    0.4051
    -5.3175    0.4051
    -6.1757    0.4480];
pts(:,2) = -pts(:,2) ;
pts = sc*pts + repmat([xOff,yOff],size(pts,1),1);
end