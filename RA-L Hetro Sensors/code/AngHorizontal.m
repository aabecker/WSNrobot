function [a] = AngHorizontal(theta1,theta2)

B =[cos(theta2),0,sin(theta2); %Pitch
    0,1,0;
    -sin(theta2),0,cos(theta2)];
A =[1,0,0;                     %Roll
    0,cos(theta1),-sin(theta1);
    0,sin(theta1),cos(theta1)];

C =A*B;

a =C(3,3);

end

