
function [OpDirMinSpanTree] = DirectedMinimumSpanTree(b,sink,q)
%BFS performed to make the Minimum Spanning Tree a Directed Minimum
%Spanning Tree. It is a recursive algorithm.

for p= 1:length(b)
    if b(p,1)==sink&&q(p,1)~=1
        x=b(p,1);
        
        b(p,1)=b(p,2);
        
        b(p,2)=x;
        
        q(p,1)=1;
        
    b = DirectedMinimumSpanTree(b,b(p,1),q);    
    end
end    
OpDirMinSpanTree=b;
end

