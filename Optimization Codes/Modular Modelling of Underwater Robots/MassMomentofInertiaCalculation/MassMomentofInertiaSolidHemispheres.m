function [Mrb_hc,I_hc] = MassMomentofInertiaSolidHemispheres(r_o,rho)
% Estimating the mass and moment of inertia for the hollow hemisphere
% r_o is the outer radius, r_i is the inner radius, h is the length

% The mass of the robot
m=2/3*rho*pi*r_o^3;

% inertia term of the matrix 

I_x=0.4*m*r_o^2;

I_y=83/320*m*r_o^2;

I_z=I_y;

Mrb_hc=m*eye(3);

I_hc=diag([I_x,I_y,I_z]);

end

