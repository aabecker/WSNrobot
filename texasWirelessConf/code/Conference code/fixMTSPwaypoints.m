function newWaypoints = fixMTSPwaypoints(waypoints,nRobots,G)
%  Calls mTSP to improve an existing collection of waypoints.  This
%  algorithm only rearranges the order of waypoints.  It assigns equal
%  numbers of waypoints to each robot
% INPUTS:  waypoints:  a mx2 matrix of waypoints.  It assumes that the
% origin is at [0,0], and that there are [0,0] at the start of each robot
% section.
%
%  Calls the function mtspf_ga.m, by Joseph Kirk  02 Sep 2008 (Updated 06 May 2014)
% http://www.mathworks.com/matlabcentral/fileexchange/21299
%
%  Aaron Becker and Srikanth KVS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if nargin< 2  %if no inputs are given, make up random waypoints
    wp_per_robot = 10;
    nRobots = 5;
    waypoints = rand(nRobots*(wp_per_robot+1),2);
    for i = 1:nRobots
        waypoints((i-1)*(wp_per_robot+1)+1,:) = [0,0];
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Recording the simulation
myConfig.xy = waypoints;


% remove the zeros rows
myConfig.xy(all(myConfig.xy==0,2),:)=[];
myConfig.xy = [ 0,0; myConfig.xy]; %re-add the zero start position
wp = myConfig.xy;
n = size(wp,1);
myConfig.dmat        = (repmat(wp(:,1),1,n)-repmat(wp(:,1)',n,1)).^2 + (repmat(wp(:,2),1,n) - repmat(wp(:,2)',n,1)).^2;

myConfig.nSalesmen   = nRobots;  % the number of robots we have
myConfig.minTour     = (size(myConfig.xy,1) - 1)/myConfig.nSalesmen;
myConfig.popSize     = 500;
myConfig.numIter     = 10e2;
myConfig.showProg    = true;
myConfig.showResult  = true;
myConfig.showWaitbar = true;
myConfig.G =  G;

%     USERCONFIG (structure) with zero or more of the following fields:
%     - XY (float) is an Nx2 matrix of city locations, where N is the number of cities
%     - DMAT (float) is an NxN matrix of city-to-city distances or costs
%     - NSALESMEN (scalar integer) is the number of salesmen to visit the cities
%     - MINTOUR (scalar integer) is the minimum tour length for any of the
%         salesmen, NOT including the start/end point
%     - POPSIZE (scalar integer) is the size of the population (should be divisible by 8)
%     - NUMITER (scalar integer) is the number of desired iterations for the algorithm to run
%     - SHOWPROG (scalar logical) shows the GA progress if true
%     - SHOWRESULT (scalar logical) shows the GA results if true
%     - SHOWWAITBAR (scalar logical) shows a waitbar if true

resultsStruct = mtspf_ga(myConfig);
%resultsStruct = mtspf_gaSectors(myConfig);
% Output:
%     RESULTSTRUCT (structure) with the following fields:
%         (in addition to a record of the algorithm configuration)
%     - OPTROUTE (integer array) is the best route found by the algorithm
%     - OPTBREAK (integer array) is the list of route break points (these specify the indices
%         into the route used to obtain the individual salesman routes)
%     - MINDIST (scalar float) is the total distance traveled by the salesmen

%%%%%%%%%%% TRANSLATE OUTPUT SO WE CAN USE FOR OTHER CODE %%%%%%%

resultsStruct.optRoute
resultsStruct.optBreak

%convert output into a list of waypoints (with zeros)
optimalRuteWZeros = resultsStruct.optRoute;
for i = numel(resultsStruct.optBreak):-1:1
    optimalRuteWZeros = [optimalRuteWZeros(1:resultsStruct.optBreak(i)),1,optimalRuteWZeros(resultsStruct.optBreak(i)+1:end)];
end
optimalRuteWZeros =[1,optimalRuteWZeros]; %insert initial position.
newWaypoints = myConfig.xy(optimalRuteWZeros,:);
display(newWaypoints)

% check by redrawing

figure
subplot(1,2,1)
plotwaypoints(waypoints, nRobots)
title('Before')
subplot(1,2,2)
plotwaypoints(newWaypoints, nRobots)
title('After')
end

function plotwaypoints(wps, nRobots)
colors = jet(nRobots);
for ct = 1:nRobots
    pt = numel(wps(:,1))/nRobots ;
    hold on
    plot([wps((ct-1)*pt+(1:pt),1);wps(1,1)],...
        [wps((ct-1)*pt+(1:pt),2);wps(1,2)],'o-','color',colors(ct,:));
    hold off

end
xlabel('x');ylabel('y');axis equal
axis tight
end
