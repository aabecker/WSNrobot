function [] = AutoSeismic_DroneHex(x,y,T,hex)
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
    y = 100;
    T = [50,0];
    hex = 10; %total number of Hexapod walkers
    drones = 10; %total number of Drones
    darts = 40; %total number of Darts
    drone_cap = 4;
end
S.x = x; S.y = y; S.hex = hex; S.drones = drones; S.darts = darts; S.drone_cap = drone_cap;
% constants
gridSpacing = 10; %meters between grid points
dt = 1; % delta T
S.minSensorsForShot = 10; %minimum number of ready sensors before a shot is taken.
S.numShots = 0;

%initialize survey points
[xg,yg] = meshgrid(0:gridSpacing:x,0:gridSpacing:y);
S.surveyPtsX = xg(:);
S.surveyPtsY = yg(:);
S.surveyPtsState = zeros(size(S.surveyPtsX));
%state: 0 no measurement, 1 assigned, 2 hex sensor at point & no measurement,
%3 dart sensor at point & no measurement, 4 measured with hex sensor, 5
%measured with dart sensor, 6 measured no sensor at location


%Initialize truck
S.Tstate = 2; % 0 - unassigned 1 - assigned 2-ready (at position) 3 - finished
S.Tpos = T;
S.Tstart = T;
S.Tgoal = T;
S.Tvel = 1;

%initialize hex
S.Hstate = zeros(hex,1); % 0 - unassigned 1 - assigned 2-ready (at position)
S.Hpos = repmat( T, hex,1);
S.Hstart = S.Hpos;
S.Hgoal = S.Hpos;
S.Hvel = 0.20;
S.Hsurveypt = zeros(hex,1);
%initilize drones
S.Dstate = zeros(drones,1); % 0- unassigned 1- deploy mode 2- retrieve mode 3- return
S.Dpos = repmat( T, drones,1);
S.Ddartscap = zeros(drones,1);
S.Dstart = S.Dpos;
S.Dgoal = S.Dpos;
S.Dvel = 0.8;
S.Dsurveypt = zeros(drones,1);
%initialize darts
S.Dastate = zeros(darts,1);% 0- unassigned 1- assigned 2-ready (at position) 3-ready to return
S.Dapos = repmat( T, darts,1);
S.Dastart = S.Dapos;
S.Dagoal = S.Dapos;
S.Dasurveypt = zeros(darts,1);
G = Init_Graphics(S);
for T = 1:dt:100000 % Main_loop
    S = Update_State(S,dt);
    G = Update_Graphics(S,G,T);
    if isFinished(S)
        break
    end
end
title(['finished in T = ',num2str(T),' seconds'])
end

function isDone = isFinished(S)
isDone = (numel(S.surveyPtsState) == sum(S.surveyPtsState == 4) + sum(S.surveyPtsState == 5) + sum(S.surveyPtsState == 6));
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

end
function S = shotTest(S)
%state: 0 no measurement, 1 assigned, 2 hex sensor at point & no measurement,
%3 dart sensor at point & no measurement, 4 measured with hex sensor, 5 measured with dart sensor.
readyPts = sum(S.surveyPtsState == 2) + sum(S.surveyPtsState == 3);
if readyPts >= S.minSensorsForShot ||  sum(S.surveyPtsState == 0)+sum(S.surveyPtsState == 1)==0
    S.numShots = S.numShots+1;
    S.surveyPtsState(S.surveyPtsState == 2) = 4;
    S.surveyPtsState(S.surveyPtsState == 3) = 5;
    S.Hstate(S.Hstate == 2) = 0;
    S.Dastate(S.Dastate == 2) = 3;
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
    if S.Hstate(k) == 0
        minDist = 10*S.x*S.y; %a very large number
        minInd = NaN;
        for m = 1:numel(S.surveyPtsX)
            if S.surveyPtsState(m) == 0 %no measurement & unassigned
                dist = sqrt(sum((S.Hpos(k,:) - [S.surveyPtsX(m),S.surveyPtsY(m)]).^2))+...
                    sqrt(sum((S.Tpos - [S.surveyPtsX(m),S.surveyPtsY(m)]).^2));
                if dist<minDist
                    minInd = m;
                    minDist = dist;
                end
            end
        end
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

function S = moveDrone(S,dt) %move Drone toward goal position
% 0- unassigned 1- deploy mode 2- retrieve mode 3- return
for k = 1:S.drones
    
    if S.Dstate(k) == 1
            if S.Ddartscap(k,1) < S.drone_cap
                dxy = sqrt(sum((S.Dgoal(k,:) - S.Dpos(k,:)).^2));
                if dxy > S.Dvel*dt
                    S.Dpos(k,:) = S.Dpos(k,:)+S.Dvel*dt*(S.Dgoal(k,:) - S.Dpos(k,:))/dxy;
                else
                    S.Dpos(k,:) = S.Dgoal(k,:);
                    S.Dstate(k) = 0;  %Unassigned so that the drone can go to it's next location for deployment
                    S.surveyPtsState(S.Dsurveypt(k)) = 3; %mark survey ready for test (dart)
                    S.Ddartscap(k,1) = S.Ddartscap(k,1) + 1;
                end   
            elseif S.Ddartscap(k,1) == S.drone_cap
                    S.Dstate(k) = 3;
%                     S.Da_f = S.Da_f + 4;
                     %S.Dastate(k:k+3,1) = 2;
                    
            end
    elseif S.Dstate(k) == 2
         if S.Ddartscap(k,1) < S.drone_cap
                dxy = sqrt(sum((S.Dgoal(k,:) - S.Dpos(k,:)).^2));
                if dxy > S.Dvel*dt
                    S.Dpos(k,:) = S.Dpos(k,:)+S.Dvel*dt*(S.Dgoal(k,:) - S.Dpos(k,:))/dxy;
                else
                    S.Dpos(k,:) = S.Dgoal(k,:);
                    S.Dstate(k) = 0;  %Unassigned so that the drone can go to it's next location for retrieval
                    S.surveyPtsState(S.Dsurveypt(k)) = 6; %mark survey ready for test (dart)
                    S.Ddartscap(k,1) = S.Ddartscap(k,1) + 1;
                end   
         elseif S.Ddartscap(k,1) == S.drone_cap
                    S.Dstate(k) = 3;
%                     S.Da_f = S.Da_f - 4;
                    %S.Dastate(k:k+3,1) = 3;
         end
    elseif S.Dstate(k) == 3
        dxy_r = sqrt(sum((S.Tgoal - S.Dpos(k,:)).^2));
                    if dxy_r > S.Dvel*dt
                    S.Dpos(k,:) = S.Dpos(k,:)+S.Dvel*dt*(S.Tgoal - S.Dpos(k,:))/dxy_r;
                    else
                    S.Dpos(k,:) = S.Tgoal;
                    S.Dstate(k) = 0;  %drone returns home it is unassigned
                    S.Ddartscap(k,1) = 0;
                    %S.Dastate(k:k+3,1) = 0;
                    
                    end
    end      
end
     
end


function S = assignDrone(S)
% for each unassigned drone, assign to the nearest unassigned survey
% point 
% a = nnz(S.surveyPtsState==3); 
% b = nnz(S.surveyPtsState==5);
for k = 1:S.drones  % 0- unassigned 1- deploy mode 2- retrieve mode 3- return
   
    if S.Dstate(k) == 0 && S.Ddartscap(k,1) < S.drone_cap
       % S.Dastate(k:k+3,1) = 1;
        minDist = 10*S.x*S.y; %a very large number
        minInd = NaN;
        for m = 1:numel(S.surveyPtsX)
            if S.surveyPtsState(m) == 0 %no measurement & unassigned
                dist = sqrt(sum((S.Dpos(k,:) - [S.surveyPtsX(m),S.surveyPtsY(m)]).^2))+...
                    sqrt(sum((S.Tpos - [S.surveyPtsX(m),S.surveyPtsY(m)]).^2));
                if dist<minDist
                    minInd = m;
                    minDist = dist;
                end
            end
        end   
           
        if ~isnan(minInd)
            S.Dgoal(k,:) = [S.surveyPtsX(minInd),S.surveyPtsY(minInd)];
            S.surveyPtsState(minInd) = 1; %assigned
            S.Dstate(k) = 1; %deployment
            S.Dstart(k,:) = S.Dpos(k,:);
            S.Dsurveypt(k) = minInd;
        end
            
%     elseif S.Dstate(k) == 0 && S.Ddartscap(k,1) < S.drone_cap
%         minDist = 10*S.x*S.y; %a very large number
%         minInd = NaN;
%         for m = 1:numel(S.surveyPtsX)
%             if S.surveyPtsState(m) == 5 %measurement taken using a smart dart sensor
%                 dist = sqrt(sum((S.Dpos(k,:) - [S.surveyPtsX(m),S.surveyPtsY(m)]).^2))+...
%                     sqrt(sum((S.Tpos - [S.surveyPtsX(m),S.surveyPtsY(m)]).^2));
%                 if dist<minDist
%                     minInd = m;
%                     minDist = dist;
%                 end
%             end
%         end
%         if ~isnan(minInd)
%             S.Dgoal(k,:) = [S.surveyPtsX(minInd),S.surveyPtsY(minInd)];
%             S.surveyPtsState(minInd) = 6; %survey done and dart returned
%             S.Dstate(k) = 2; %retrieval
%             S.Dstart(k,:) = S.Dpos(k,:);
%             S.Dsurveypt(k) = minInd;
%             S.darts = S.darts - 1;
%            
%         end
    elseif S.Dstate(k) == 0 && S.Ddartscap(k,1) == S.drone_cap
            S.Dgoal(k,:) = [S.Tgoal(:,1),S.Tgoal(:,2)];
            S.Dstate(k) = 3; %return
            S.Dstart(k,:) = S.Dpos(k,:);
        
%     elseif S.Dstate(k) == 0 && S.Ddartscap(k,1) == S.drone_cap
%             S.Dgoal(k,:) = [S.Tgoal(:,1),S.Tgoal(:,2)];
%             S.Dstate(k) = 3; %return
%             S.Dstart(k,:) = S.Dpos(k,:);
           
            
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
        tpts = truckPts(S.Tpos(1),S.Tpos(2),1);
        G.truck = fill(tpts(:,1),tpts(:,2),'b');
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
            G.DPathG(k) = line([S.Dstart(k,1),S.Dgoal(k,1)],[S.Dstart(k,2),S.Dgoal(k,2)],'color' ,'g');
        end
        for k = 1:S.drones    %draw current path
            G.DPathP(k) = line([S.Dpos(k,1),S.Dgoal(k,1)],[S.Dpos(k,2),S.Dgoal(k,2)],'color' ,'m');
        end
        for k = 1:S.drones    %draw the drones
            dpts = dPts(S.Dpos(k,1),S.Dpos(k,2),1);
            G.d(k) = fill(dpts(:,1),dpts(:,2),'k');
        end
        %draw darts
        G.da = ones(S.darts,1);
        for k = 1:S.darts     %draw the darts
            dapts = daPts(S.Dapos(k,1),S.Dapos(k,2),1);
            G.da(k) = fill(dapts(:,1),dapts(:,2),'y');
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
            dpts = dPts(S.Dpos(k,1),S.Dpos(k,2),1);
            set( G.d(k), 'XData',dpts(:,1) ,'YData',dpts(:,2));
            set( G.DPathG(k), 'XData',[S.Dstart(k,1),S.Dgoal(k,1)],'YData',[S.Dstart(k,2),S.Dgoal(k,2)]);
            set( G.DPathP(k), 'XData',[S.Dpos(k,1),S.Dgoal(k,1)],'YData',[S.Dpos(k,2),S.Dgoal(k,2)]);
            
        end
        %draw darts
        for k = 1:S.darts
            dapts = daPts(S.Dapos(k,1),S.Dapos(k,2),1);
            set( G.da(k), 'XData',dapts(:,1) ,'YData',dapts(:,2));
        end
        %draw people
        
        %update title
        title(['T = ',num2str(T),', ',num2str(sum(S.surveyPtsState == 2)),' pts ready, ',num2str(S.numShots),' shots'])
        drawnow;
    end % end update graphics

    function pts = hexPts(xOff,yOff,sc)
        th = 0:pi/3:2*pi;
        pts = sc*[xOff+cos(th)',yOff+sin(th)'];
    end

    function pts = dPts(xOff,yOff,sc)
        th = 0:pi/3:2*pi;
        pts = sc*[xOff+cos(th)',yOff+sin(th)'];
    end

    function pts = daPts(xOff,yOff,sc)
        th = 0:pi/3:2*pi;
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
