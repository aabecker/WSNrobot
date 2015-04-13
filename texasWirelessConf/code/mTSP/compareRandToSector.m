% By starting with salesmen assigned to certain sectors, I get much 80%
% shorter routes.  This s an effective heuristic for priming the search.

L = 1000;
n= 100;
nSalesmen = 5;

myConfig.dmat        = [];
myConfig.nSalesmen   = nSalesmen;
myConfig.minTour     = floor(1/4*(n/myConfig.nSalesmen));
myConfig.popSize     = 80;
myConfig.numIter     = 1e3;
myConfig.showProg    = true;
myConfig.showResult  = true;
myConfig.showWaitbar = true;

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

nTests = 5;
minDistRand = zeros(1,nTests);
minDistSect = zeros(1,nTests);

for j  = 1:nTests
    myConfig.xy          = L*rand(n,2)- repmat(L/2*[1,1],n,1);
    myConfig.xy(1,:) = [0,0];
    resultsStructSect = mtspf_gaSectors(myConfig);
    resultsStructRand = mtspf_ga(myConfig);
   
    minDistRand(j) = resultsStructRand.minDist;
    minDistSect(j) = resultsStructSect.minDist;
    display([num2str(j),') rand Start = ',num2str( minDistRand(j)),', theta start = ',num2str( minDistSect(j))])
    
end

display(['MEAN rand Start = ',num2str(mean(minDistRand)),', MEAN theta start = ',num2str(mean(minDistSect))])

save(['CompareRandSec',num2str( myConfig.nSalesmen),'Points',num2str(n),'.mat'],'minDistRand','minDistSect');



% Output:
%     RESULTSTRUCT (structure) with the following fields:
%         (in addition to a record of the algorithm configuration)
%     - OPTROUTE (integer array) is the best route found by the algorithm
%     - OPTBREAK (integer array) is the list of route break points (these specify the indices
%         into the route used to obtain the individual salesman routes)
%     - MINDIST (scalar float) is the total distance traveled by the salesmen