C_E_1=eye(12);
D_E_1=zeros(12,6);
Plant_E1=ss(A_E_1,B_E_1,C_E_1,D_E_1)

Ts=0.2;
p=5;
m=2

mpcobj_E1 = mpc(Plant_E1,Ts,p,m)



mpcobj_E1.Weights=struct('MV',[1 1 1 1 1 1],'MVRate',[.1 .1 .1 .1 .1 .1],'OV',[1 1 1 1 1 1 1 1 1 1 1 1]);

get(mpcobj_E1)
