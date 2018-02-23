function B_d = DirObjFormulation(u,b_T,lambda,r_T)

% This function formulate the optimization of each direction vector

% u is a scalar: the desired input for this thruster

% b_T is the spin direction :{-1,1}

% r_T is three dimensional position vector r_T=[r1,r2,r3]

r1=r_T(1);

r2=r_T(2);

r3=r_T(3);

B_d=u*[eye(3);b_T*lambda,-r3,r2;r3,b_T*lambda,-r1;-r2,r1,b_T*lambda];



end

