figure;
axis equal;
x1=[-2,-2;
    -3,0;
    1,-2;
    6,-4;
    -2,0;
    -1,3;
    -4,-1;
    0,-1;
    -2,0;
    -1,-1];
X1 =ones(length(x1),1);
for i =1:length(x1)
    X1(i,1) = AngHorizontal(x1(i,1),x1(i,2));
end
X1 =abs(X1);
x2=[-1,0;
    -2,2;
    0,2;
    -3,1;
    -2,0;
    -3,0;
    -1,-2;
    -5,-1;
    -4,0;
    -3,-1];
X2 =ones(length(x2),1);
for i =1:length(x2)
    X2(i,1) = AngHorizontal(x2(i,1),x2(i,2));
end
X2 =abs(X2);
x3=[-1,1;
    -3,-4;
    0,-5;
    -3,-1;
    3,-2;
    -2,1;
    0,1;
    -2,-1;
    -1,0;
    -1,1];

X3 =ones(length(x3),1);
for i =1:length(x3)
    X3(i,1) = AngHorizontal(x3(i,1),x3(i,2));
end
X3 =abs(X3);
boxplot([X1,X2,X3],'plotstyle','traditional','boxstyle','outline','colors','rgb','labels',{'sand','grass','dry clay'})