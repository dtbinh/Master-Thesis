function Ma_C = AddedMassCylinder(m,rho,r,L)
% This function calculates the added mass of a cylinder
% two parameters decide the geometry of a cylinder

% r: radium 
% L: length 

% V: volume of the cylinder
% rho: density of the fluid

V=pi*r^2*L;
 

% according to the slender body theory
m11=0.1*m;
m22=rho*V;
m33=m22;
m44=0;
m55=(L^2)/12*rho*V;
m66=m55;

Ma_C=diag([m11 m22 m33 m44 m55 m66]);

end

 