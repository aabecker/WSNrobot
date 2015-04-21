function callWaypointCodeAndMtsp()
set(0,'defaultaxesfontsize',32);
set(0,'defaulttextfontsize',32);
nRobots = 4;
 [x1,y1] = hilbert(2);
 waypoints1=[0 ,0;10*x1'+5,10*y1'+5];
 waypoints2=[0 ,0;10*x1'-5,10*y1'-5];
 waypoints3=[0 ,0;10*x1'+5,10*y1'-5];
 waypoints4=[0 ,0;10*x1'-5,10*y1'+5];
 WAYPOINTS=[waypoints1;waypoints2;waypoints3;waypoints4];
  MOVIE_NAME = 'FourRobots2';
 
 nRobots = 1;
  [x1,y1] = hilbert(3);
   WAYPOINTS=[0 ,0;20*x1',20*y1'];
  MOVIE_NAME = 'OneRobot';

 iterationStart = 0;
 MAKE_MOVIE = true;
 
     G.writerObj = VideoWriter(MOVIE_NAME,'MPEG-4');%http://www.mathworks.com/help/matlab/ref/videowriterclass.html
    set(G.writerObj,'Quality',100);
    open(G.writerObj);
 
 
[WAYPOINTS,costs,iterationEnd,G] = MultiRobotWaypointControlForPathPlanningTWMCS(MOVIE_NAME, iterationStart,MAKE_MOVIE,WAYPOINTS,nRobots,G);
WAYPOINTS = fixMTSPwaypoints(WAYPOINTS,nRobots, G)
[WAYPOINTS,costs,iterationEnd,G] = MultiRobotWaypointControlForPathPlanningTWMCS(MOVIE_NAME, iterationStart+200,MAKE_MOVIE,WAYPOINTS,nRobots,G);
WAYPOINTS = fixMTSPwaypoints(WAYPOINTS,nRobots, G)


close(G.writerObj);

end