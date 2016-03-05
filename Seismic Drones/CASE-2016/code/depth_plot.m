figure;
axis equal;
x1=[40.57,43.37,31.45,50.20,29.86,18.86,34.93,36.36,45.66,42.06]';
x2=[10.03,7.93,8.83,8.47,8.18,9.79,11.33,9.12,11.80,12.02]';
x3=[0,0,0,0,3,1,0,0,1,2]';
boxplot([x1,x2,x3],'plotstyle','traditional','boxstyle','outline','colors','rgb','labels',{'sand','grass','dry clay'})