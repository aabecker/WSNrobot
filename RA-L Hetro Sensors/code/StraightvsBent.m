% plots experiment data for straight and twisted fins

% angle St,  pen st, ang bent, pen bent
A =[80.15	67.13	84.53	71.62;
87.6	75.3	95.74	72.97;
88.13	84.13	81.38	79.64;
78.2	83.09	86.93	63.39;
94.12	86.49	84.72	85.08;
92.26	81.53	84.7	88;
76.15	84.37	85.99	63.53;
84.53	81.09	95.16	86.12;
79.19	78.75	84.45	88.93;
100.57	80.5	83.98	84.43];

dffontsz= 16;
set(0,'DefaultLineLineWidth',2)
set(0,'defaultaxesfontsize',dffontsz);
set(0,'defaulttextfontsize',dffontsz);
rv = 0.2+0.02*linspace(-1,1,10)';
figure(1); clf;
subplot(1,2,1)
 A1=[ones(10,1),abs(A(:,1)-90)];  %  straight angle
 A2=[2*ones(10,1),abs(A(:,3)-90)];%  bent angle
 bh = boxplot([abs(A(:,1)-90),abs(A(:,3)-90)],'Labels',{'Straight Fins','Twisted Fins'});
  set(bh(:),'linewidth',1); %make lines thick
 hold on;
 plot(A1(:,1)+rv,A1(:,2),'rO');
 plot(A2(:,1)+rv,A2(:,2),'bO');
 ylabel 'Angle of Deviation (deg)';
subplot(1,2,2)
 A3=[ones(10,1),A(:,2)];
 A4=[2*ones(10,1),A(:,4)];
 bh2= boxplot([A(:,2),A(:,4)],'Labels',{'Straight Fins','Twisted Fins'});
   set(bh2(:),'linewidth',1); %make lines thick
 hold on;
 plot(A3(:,1)+rv,A3(:,2),'rO');
 hold on;
 plot(A4(:,1)+rv,A4(:,2),'bO');
 ylabel 'Penetration Depth (mm)';
 
 
 set(gcf,'PaperUnits','inches')
set(gcf,'papersize',[8.5,3])
set(gcf,'paperposition',[-0.7,0,10,3])

%%% SAVE THE PICTURE?
print -dpdf '../pictures/StraightvsTwistedAngleDepth.pdf'
%display('finished plotting')
 