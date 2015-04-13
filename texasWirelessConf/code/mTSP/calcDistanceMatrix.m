 
n =5
wp = rand(n,2);

dmat = (repmat(wp(:,1),1,n)-repmat(wp(:,1)',n,1)).^2 + (repmat(wp(:,2),1,n) - repmat(wp(:,2)',n,1)).^2;


