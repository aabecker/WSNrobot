function [ directednodesfinal ] = DirectedMinSpanningTree(b,sink)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

m=[b(:,1);b(:,2)];
y1 = zeros(size(m));
T=[];
test=zeros(length(b),1);
for i1 = 1:length(m)
y1(i1) = sum(m==m(i1));
if y1(i1)==1
T=[T;i1]; %position of single nodes i.e. connected only to one other node in the WSN
end
end
% disp(T);
T1=[];
for i2 = 1:length(T)
    if T(i2)<=length(b)
        test(T(i2))=1;
    elseif T(i2)>length(b)
        r=T(i2)-length(b);
        T1=[T1;r];
    end
end 
T(5:end,1)=T1;
%  disp(b);
for i3 = 1:length(T1)
    o=b(T1(i3),1);
    p=b(T1(i3),2);
    b(T1(i3),1)=p;
    b(T1(i3),2)=o;
    if test(T1(i3))==0
        test(T1(i3))=1;
    end    
end 
b1=b;

for i4 = 1:length(m)
    if m(i4)==sink && i4<=length(b)
     o=b(i4,1);
    p=b(i4,2);
    b(i4,1)=p;
    b(i4,2)=o; 
    test(i4,1)=1;
    end
    if m(i4)==sink && i4>length(b)
     test(i4-length(b),1)=1;
    end 
end  
t=[];
for i5=1:length(b)
    if b(i5,2)==sink
        t=[t;b(i5,1)];
    end   
end
b2=b;
t1=[];
for i6=1:length(b)
    for i7=1:length(t)
        
        if t(i7,1)==b(i6,1) && b(i6,2)~=sink
            o=b(i6,1);
            p=b(i6,2);
            b(i6,1)=p;
            b(i6,2)=o;
            test(i6,1)=1;
            t1=[t1;b(i6,1)];
        end
    end
end
t2=[];
for i8=1:length(b)
    for i9=1:length(t1)
        
        if t1(i9,1)==b(i8,1) && test(i8,1)==0
            o=b(i8,1);
            p=b(i8,2);
            b(i8,1)=p;
            b(i8,2)=o;
            test(i8,1)=1;
            t2=[t2;b(i8,1)];
        end
        
    end
end    
    t3=[];
for i10=1:length(b)
    for i11=1:length(t1)
        
        if t2(i11,1)==b(i10,1) && test(i10,1)==0
            o=b(i10,1);
            p=b(i10,2);
            b(i10,1)=p;
            b(i10,2)=o;
            test(i10,1)=1;
            t3=[t3;b(i10,1)];
        end
        
    end
end    
    
%  unqm=unique(m);
% countElm=histc(m,unqm); 
%  unqb=unique(b(:,2));
%  countElb=histc(b(:,2),unqb);
a=0;
for i12=1:length(b)
    if test(i12,1)==0
for i13=1:length(b)
        if b(i12,1)==b(i13,2)
            a=a+1;
        end
        if a >= 1
            test(i12,1)=1;
        end
end

    end
end    
directednodesfinal=b;
end