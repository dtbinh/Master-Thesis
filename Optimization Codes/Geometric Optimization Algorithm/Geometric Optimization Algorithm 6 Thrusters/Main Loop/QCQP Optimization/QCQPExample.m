Q = [3,2,1;
     2,4,0;
     1,0,5];
f = [-24;-48;-130];
c = -2;

rng default % for reproducibility
% Two sets of random quadratic constraints:
H{1} = gallery('randcorr',3); % random positive definite matrix
H{2} = gallery('randcorr',3);
k{1} = randn(3,1);
k{2} = randn(3,1);
d{1} = randn;
d{2} = randn;

[x1,x2]=QCQPOptimization(Q,f,c,H,k,d)