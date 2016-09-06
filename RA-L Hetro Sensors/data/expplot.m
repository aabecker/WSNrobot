
for A =70:5:110
X =1:1:100;
Y = A*(1-exp(-0.3*X));
plot(X,Y);
hold on;
end
hold on

x_val = [0,4,10,15,40,80];
y_val = [40.536,59.477,71.202,86.598,94.004,106.554];
plot (x_val,y_val,'bO');
xlim([-10 100]);
%'MarkerSize',15,'MarkerEdgeColor',[0 0 0],'MarkerFaceColor', [0 0 0]