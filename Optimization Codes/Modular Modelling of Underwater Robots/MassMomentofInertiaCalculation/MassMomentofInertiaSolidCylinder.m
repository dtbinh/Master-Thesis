function [Mrb_sc,I_sc] = MassMomentofInertiaSolidCylinder(r_o,h,rho)
% Estimating the mass and moment of inertia for the solid cylinder
% The thruster housings and batteries are modeled as solid cylinders 
% r_o is the radius, h is the length

% The mass of the robot
m=rho*pi*r_o^2*h;

% inertia term of the matrix 

I_x=0.5*m*r_o^2;

I_y=(1/12)*m*(3*r_o^2+h^2);

I_z=I_y;

Mrb_hc=m*eye(3);

I_hc=diag([I_x,I_y,I_z]);



end

