axis 'equal';
x1 = [1,53.27;
             0.98,38.36;
             0.98,33.88;
             1,60.88;
             1.02,43.91;
             1.02,37.04;
             0.98,43.64;
             1,33.84;
             1,28.50;
             1.02,32.04];
         
x2 = [2.02,57.75;
2,63.41;
1.98,57.62;
2.02,55.33;
1.98,63.67;
2,66.84;
2,52.20;
2.02,61.39;
2,60.48;
2,56.08];

x3 = [3,63.48;
3,82.52;
3,88.67;
2.98,66.89;
3.02,66.61;
2.98,62.14;
3,65.57;
3,72.05;
3.01,65.26;
3,78.83];

x4 = [3.98,97.19;
4,86.9;
4.02,88.36;
3.99,84.49;
4.01,86.90;
3.99,87.04;
4,82.21;
4.02,87.77;
4.01,83.44;
3.99,81.66];

x5 = [4.98,90.14;
5,89.67;
5.02,90.98;
5.01,87.23;
4.99,99.74;
4.98,98.59;
5,99.49;
5.02,96.11;
5.01,99.09;
4.99,89];

x6 = [5.98,98.4;
6,107.1;
6.02,107.86;
6.01,102.42;
5.99,102.41;
5.98,111.11;
6,105.21;
6.02,111.49;
6.01,110.53;
5.99,109.01];

figure(1);
h1 = [1,sum(x1(:,2))/10,std(x1(:,2));
      2,sum(x2(:,2))/10,std(x2(:,2));
      3,sum(x3(:,2))/10,std(x3(:,2));
      4,sum(x4(:,2))/10,std(x4(:,2));
      5,sum(x5(:,2))/10,std(x5(:,2));
      6,sum(x6(:,2))/10,std(x6(:,2))];
plot (h1(:,1),h1(:,2),'mO-','MarkerSize',10,'MarkerEdgeColor',[0 0 0],'MarkerFaceColor', [1 0 1]);
hold on;
errorbar(h1(:,2),h1(:,3),'m-','LineWidth',2);
hold on;
% boxplot([x1(:,2),x2(:,2),x3(:,2),x4(:,2),x5(:,2),x6(:,2)],'Labels',{'h = 0','h = 5','h = 10','h = 15','h = 40','h = 80'});
xlabel 'Drop Height (mm)';
ylabel 'Pentration Depth (mm)';
% title 'Masons Sand';
 hold on;
plot (x1(:,1)-0.1,x1(:,2),'kO','MarkerSize',4,'MarkerEdgeColor',[0 0 0],'MarkerFaceColor', [1 0 1]);
hold on;
plot (x2(:,1)-0.1,x2(:,2),'kO','MarkerSize',4,'MarkerEdgeColor',[0 0 0],'MarkerFaceColor', [1 0 1]);
hold on;
plot (x3(:,1)-0.1,x3(:,2),'kO','MarkerSize',4,'MarkerEdgeColor',[0 0 0],'MarkerFaceColor', [1 0 1]);
hold on;
plot (x4(:,1)-0.1,x4(:,2),'kO','MarkerSize',4,'MarkerEdgeColor',[0 0 0],'MarkerFaceColor', [1 0 1]);
hold on;
plot (x5(:,1)-0.1,x5(:,2),'kO','MarkerSize',4,'MarkerEdgeColor',[0 0 0],'MarkerFaceColor', [1 0 1]);
hold on;
plot (x6(:,1)-0.1,x6(:,2),'kO','MarkerSize',4,'MarkerEdgeColor',[0 0 0],'MarkerFaceColor', [1 0 1]);
hold on;

a1 = [0.98,35.82;
1,44.57;
1.02,60.35;
1.01,32.74;
0.99,39.96;
0.98,55.49;
1,26.55;
1.02,36.97;
1.01,46.02;
0.99,44.17];
         
a2 = [1.98,62.42;
2,64.42;
2.02,55.73;
1.99,33.7;
2.01,43.64;
1.98,38.69;
2,47.34;
2.02,35.46;
2.01,48.01;
1.99,36.32];

a3 = [2.99,64.88;
2.98,44.14;
3.02,45.09;
3.01,47.89;
3.02,49.02;
3.01,42.99;
2.99,46.83;
2.98,48.22;
3,47.15;
3,49.3];

a4 = [4,67.4;
4,63.63;
4.02,56.81;
4.02,59.79;
3.98,72.19;
3.98,58.39;
3.99,64.72;
3.99,72.99;
4.01,61.68;
4.01,63.86];

a5 = [5,87.28;
5,75.3;
5.02,84.19;
5.01,72.84;
5.02,87.64;
5.01,86.74;
4.98,77.69;
4.98,72.91;
4.99,84.98;
4.99,77];

a6 = [6,99.48;
6,89.33;
6.02,98.24;
6.01,90.8;
6.01,95.51;
6.02,96.37;
5.98,100.67;
5.98,96.29;
5.99,91.59;
5.99,87.93];

figure(1);
h2 = [1,sum(a1(:,2))/10,std(a1(:,2));
      2,sum(a2(:,2))/10,std(a2(:,2));
      3,sum(a3(:,2))/10,std(a3(:,2));
      4,sum(a4(:,2))/10,std(a4(:,2));
      5,sum(a5(:,2))/10,std(a5(:,2));
      6,sum(a6(:,2))/10,std(a6(:,2))];
plot (h2(:,1),h2(:,2),'bO-','MarkerSize',10,'MarkerEdgeColor',[0 0 0],'MarkerFaceColor', [0 0 1]);
hold on;
errorbar(h2(:,2),h2(:,3),'b-','LineWidth',2);
hold on;
% boxplot([a1(:,2),a2(:,2),a3(:,2),a4(:,2),a5(:,2),a6(:,2)],'Labels',{'h = 0','h = 5','h = 10','h = 15','h = 40','h = 80'});
xlabel 'Drop Height (mm)';
ylabel 'Pentration Depth (mm)';
% title 'River Sand';
hold on;
plot (a1(:,1)+0.1,a1(:,2),'kO','MarkerSize',4,'MarkerEdgeColor',[0 0 0],'MarkerFaceColor', [0 0 1]);
hold on;
plot (a2(:,1)+0.1,a2(:,2),'kO','MarkerSize',4,'MarkerEdgeColor',[0 0 0],'MarkerFaceColor', [0 0 1]);
hold on;
plot (a3(:,1)+0.1,a3(:,2),'kO','MarkerSize',4,'MarkerEdgeColor',[0 0 0],'MarkerFaceColor', [0 0 1]);
hold on;
plot (a4(:,1)+0.1,a4(:,2),'kO','MarkerSize',4,'MarkerEdgeColor',[0 0 0],'MarkerFaceColor', [0 0 1]);
hold on;
plot (a5(:,1)+0.1,a5(:,2),'kO','MarkerSize',4,'MarkerEdgeColor',[0 0 0],'MarkerFaceColor', [0 0 1]);
hold on;
plot (a6(:,1)+0.1,a6(:,2),'kO','MarkerSize',4,'MarkerEdgeColor',[0 0 0],'MarkerFaceColor', [0 0 1]);
hold on;

b1 = [1,53.64;
1,48.36;
1.02,44.42;
1.02,50.69;
1.01,57.78;
1.01,53.33;
0.98,41.88;
0.98,43.88;
0.99,52.47;
0.99,40.95];
         
b2 = [2,71.79;
2,67.52;
2.02,58.11;
1.98,59.18;
2.02,59.1;
2.02,62.26;
2.01,72.3;
1.98,53.18;
1.99,61.19;
1.99,69.25];

b3 = [3,65.03;
3,74.02;
3.02,63.07;
3.01,61.69;
3.02,74.95;
3.01,64.5;
2.98,73.94;
2.98,62.84;
2.99,74.85;
2.99,67.15];

b4 = [4,74.27;
4,70.07;
4.02,71.4;
4.02,77.11;
4.01,72.21;
4.01,69.7;
3.98,76.49;
3.98,73.24;
3.99,70;
3.98,68.98];

b5 = [5,84.46;
5.02,85.11;
5,82.96;
5.02,92.11;
5.01,83.57;
4.98,82.73;
4.99,82.33;
5.01,78.66;
4.99,84.53;
4.98,77.81];

b6 = [6,90.64;
6,87.33;
6.02,100.77;
6.02,97.74;
6.01,87.39;
6.01,97.24;
5.98,88.49;
5.98,98.62;
5.99,90.32;
5.99,97.03];

figure(1);
h3 = [1,sum(b1(:,2))/10,std(b1(:,2));
      2,sum(b2(:,2))/10,std(b2(:,2));
      3,sum(b3(:,2))/10,std(b3(:,2));
      4,sum(b4(:,2))/10,std(b4(:,2));
      5,sum(b5(:,2))/10,std(b5(:,2));
      6,sum(b6(:,2))/10,std(b6(:,2))];
plot (h3(:,1),h3(:,2),'rO-','MarkerSize',10,'MarkerEdgeColor',[0 0 0],'MarkerFaceColor', [1 0 0]);
hold on;
errorbar(h3(:,2),h3(:,3),'r-','LineWidth',2);
hold on;
% boxplot([b1(:,2),b2(:,2),b3(:,2),b4(:,2),b5(:,2),b6(:,2)],'Labels',{'h = 0','h = 5','h = 10','h = 15','h = 40','h = 80'});
xlabel 'Drop Height (mm)';
ylabel 'Pentration Depth (mm)';
% title 'Coarse Sand';
hold on;
plot (b1(:,1),b1(:,2),'kO','MarkerSize',4,'MarkerEdgeColor',[0 0 0],'MarkerFaceColor', [1 0 0]);
hold on;
plot (b2(:,1),b2(:,2),'kO','MarkerSize',4,'MarkerEdgeColor',[0 0 0],'MarkerFaceColor', [1 0 0]);
hold on;
plot (b3(:,1),b3(:,2),'kO','MarkerSize',4,'MarkerEdgeColor',[0 0 0],'MarkerFaceColor', [1 0 0]);
hold on;
plot (b4(:,1),b4(:,2),'kO','MarkerSize',4,'MarkerEdgeColor',[0 0 0],'MarkerFaceColor', [1 0 0]);
hold on;
plot (b5(:,1),b5(:,2),'kO','MarkerSize',4,'MarkerEdgeColor',[0 0 0],'MarkerFaceColor', [1 0 0]);
hold on;
plot (b6(:,1),b6(:,2),'kO','MarkerSize',4,'MarkerEdgeColor',[0 0 0],'MarkerFaceColor', [1 0 0]);
hold on;

c1 = [0.98,0;
0.99,0;
1,0;
1.01,0;
1.02,0;
0.98,0;
0.99,0;
1,0;
1.01,0;
1.02,0];
         
c2 = [2,43.03;
2.02,39.01;
2.01,41.89;
1.97,41.05;
1.98,42.95;
2.02,42.35;
2,39.19;
1.97,48.83;
1.98,39.37;
1.99,41.91];

c3 = [3,40.87;
3,47.68;
3.02,42.88;
3.01,40.97;
3.03,42.22;
3.02,40.06;
2.99,40.81;
2.98,41.45;
2.97,40.81;
2.97,38.14];

c4 = [4,38.49;
4.02,39.25;
4,41.61;
4.02,36.73;
4.01,45.13;
4.01,40.95;
3.97,40.33;
3.97,43.57;
3.98,44.04;
3.99,44.86];

c5 = [5,56.17;
5.02,58.41;
5,67.99;
5.02,43.17;
5.01,54.17;
5.01,64.65;
4.98,56.52;
4.98,60.97;
4.99,61.58;
4.99,51.61];

c6 = [6,73.58;
6,66.63;
6.02,67.81;
6.02,77.95;
6.01,79.93;
5.98,80.7;
5.99,81.54;
6.01,71.59;
5.98,68.1;
5.99,74.29];

figure(1);
h4 = [1,sum(c1(:,2))/10,std(c1(:,2));
      2,sum(c2(:,2))/10,std(c2(:,2));
      3,sum(c3(:,2))/10,std(c3(:,2));
      4,sum(c4(:,2))/10,std(c4(:,2));
      5,sum(c5(:,2))/10,std(c5(:,2));
      6,sum(c6(:,2))/10,std(c6(:,2))];
plot (h4(:,1),h4(:,2),'gO-','MarkerSize',10,'MarkerEdgeColor',[0 0 0],'MarkerFaceColor', [0 1 0]);
hold on;
errorbar(h4(:,2),h4(:,3),'g-','LineWidth',2);
hold on;
  boxplot([c1(:,2),c2(:,2),c3(:,2),c4(:,2),c5(:,2),c6(:,2)],'Labels',{'h = 0','h = 5','h = 10','h = 15','h = 40','h = 80'});
xlabel 'Drop Height (mm)';
ylabel 'Pentration Depth (mm)';
% title 'Soil';
% hold on;
plot (c1(:,1),c1(:,2),'kO','MarkerSize',4,'MarkerEdgeColor',[0 0 0],'MarkerFaceColor', [0 1 0]);
hold on;
plot (c2(:,1),c2(:,2),'kO','MarkerSize',4,'MarkerEdgeColor',[0 0 0],'MarkerFaceColor', [0 1 0]);
hold on;
plot (c3(:,1),c3(:,2),'kO','MarkerSize',4,'MarkerEdgeColor',[0 0 0],'MarkerFaceColor', [0 1 0]);
hold on;
plot (c4(:,1),c4(:,2),'kO','MarkerSize',4,'MarkerEdgeColor',[0 0 0],'MarkerFaceColor', [0 1 0]);
hold on;
plot (c5(:,1),c5(:,2),'kO','MarkerSize',4,'MarkerEdgeColor',[0 0 0],'MarkerFaceColor', [0 1 0]);
hold on;
plot (c6(:,1),c6(:,2),'kO','MarkerSize',4,'MarkerEdgeColor',[0 0 0],'MarkerFaceColor', [0 1 0]);
hold on;

